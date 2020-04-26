import time
import sys
import tellopy
import keyboard
import pygame
import cv2
import numpy
import av
import threading
import traceback
from simple_pid import PID
import tensorflow as tf
import argparse

import posenet

parser = argparse.ArgumentParser()
parser.add_argument('--model', type=int, default=101)
parser.add_argument('--cam_id', type=int, default=0)
parser.add_argument('--cam_width', type=int, default=1280) #defaults need to be changed?
parser.add_argument('--cam_height', type=int, default=720)
parser.add_argument('--scale_factor', type=float, default=0.7125)
parser.add_argument('--file', type=str, default=None, help="Optionally use a video file instead of a live camera")
args = parser.parse_args()


from pygame.locals import *

prev_flight_data = None
run_controller_thread = True
shutdown = False
#drone control inputs
drone_cc = 0
drone_ud = 0
drone_fb = 0


def controller_thread():
    global drone
    global drone_cc
    global drone_ud
    global drone_fb
    global shutdown
    #initialize previous drone control inputs
    control_on = True #allows you to toggle control so that you can force landing
    pdrone_cc = -111
    pdrone_ud = -111
    pdrone_fb = -111

    global run_controller_thread
    print('start controller_thread()')
    try:
        while run_controller_thread:
            time.sleep(.05)
            # takeoff
            if keyboard.is_pressed('space'):
                drone.takeoff()
            # land
            elif keyboard.is_pressed('l'):
                drone.land()
                control_on = False #disable control
                shutdown = True

            elif keyboard.is_pressed('q'):
                drone.counter_clockwise(40)
            elif keyboard.is_pressed('e'):
                drone.clockwise(40)
            elif keyboard.is_pressed('d'):
                drone.right(40)
            elif keyboard.is_pressed('a'):
                drone.left(40)
            elif keyboard.is_pressed('w'):
                drone.forward(40)
            elif keyboard.is_pressed('s'):
                drone.backward(40)
            elif keyboard.is_pressed('r'):
                drone.clockwise(0)
                drone.forward(0)
                drone.left(0)
            elif keyboard.is_pressed('t'): #toggle controls
                control_on = False
            elif keyboard.is_pressed('esc'):
                drone.land()
                break

            #set commands based on PID output
            if control_on and (pdrone_cc != drone_cc):
                if drone_cc < 0:
                    drone.clockwise(int(drone_cc)*-1)
                else:
                    drone.counter_clockwise(int(drone_cc))
                pdrone_cc = drone_cc
            if control_on and (pdrone_ud != drone_ud):
                if drone_ud < 0:
                    drone.down(min([100,int(drone_ud)*-2])) #easily moving downwards requires control output to be magnified
                else:
                    drone.up(int(drone_ud))
                pdrone_ud = drone_ud
            if control_on and (pdrone_fb != drone_fb):
                if drone_fb < 0:
                    drone.backward(min([50,int(drone_fb)*-1])) #easily moving downwards requires control output to be magnified
                else:
                    drone.forward(min([50,int(drone_fb)]))
                pdrone_fb = drone_fb

    except KeyboardInterrupt as e:
        print(e)
    except Exception as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(e)
    finally:
        run_controller_thread = False

def handler(event, sender, data, **args):
    global prev_flight_data
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        if prev_flight_data != str(data):
            print(data)
            prev_flight_data = str(data)
    else:
        print('event="%s" data=%s' % (event.getname(), str(data)))

def main():
    global drone
    global drone_cc
    global drone_ud
    global drone_fb
    global shutdown
    drone = tellopy.Tello()
    drone.connect()
    drone.wait_for_connection(60.0)
    drone.start_video()

    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    pid_cc = PID(0.35,0.2,0.2,setpoint=0,output_limits=(-100,100))
    pid_ud = PID(0.3,0.3,0.3,setpoint=0,output_limits=(-80,80))
    pid_fb = PID(0.35,0.2,0.3,setpoint=0,output_limits=(-50,50))

    video = cv2.VideoWriter('test_out.mp4',-1,1,(320,240))
    # drone.subscribe(drone.EVENT_VIDEO_FRAME,handler)
    print("Start Running")
    with tf.Session() as sess:
        model_cfg, model_outputs = posenet.load_model(args.model, sess)
        output_stride = model_cfg['output_stride']

        try:
            # threading.Thread(target=recv_thread).start()
            threading.Thread(target=controller_thread).start()
            container = av.open(drone.get_video_stream())
            frame_count = 0
            while not shutdown:
                for frame in container.decode(video=0):
                    frame_count = frame_count + 1
                    # skip first 300 frames
                    if frame_count < 300:
                        continue
                    if frame_count %4 == 0:
                        im = numpy.array(frame.to_image())
                        im = cv2.resize(im, (320,240)) #resize frame
                        image = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
                        #image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                        input_image, display_image, output_scale = posenet.process_input(
                            image, scale_factor=args.scale_factor, output_stride=output_stride)

                        heatmaps_result, offsets_result, displacement_fwd_result, displacement_bwd_result = sess.run(
                            model_outputs,
                            feed_dict={'image:0': input_image}
                        )

                        pose_scores, keypoint_scores, keypoint_coords = posenet.decode_multi.decode_multiple_poses(
                            heatmaps_result.squeeze(axis=0),
                            offsets_result.squeeze(axis=0),
                            displacement_fwd_result.squeeze(axis=0),
                            displacement_bwd_result.squeeze(axis=0),
                            output_stride=output_stride,
                            max_pose_detections=10,
                            min_pose_score=0.15)

                        keypoint_coords *= output_scale

                        # TODO this isn't particularly fast, use GL for drawing and display someday...
                        overlay_image = posenet.draw_skel_and_kp(
                            display_image, pose_scores, keypoint_scores, keypoint_coords,
                            min_pose_score=0.15, min_part_score=0.1)



                        centerx = int(overlay_image.shape[1]/2)
                        centery = int(overlay_image.shape[0]/2)
                        nosex = int(keypoint_coords[0,0,1])
                        nosey = int(keypoint_coords[0,0,0])
                        ctrl_out_cc = 0
                        ctrl_out_ud = 0

                        #overlay_image = cv2.putText(overlay_image, str(nosey), (120,50), cv2.FONT_HERSHEY_SIMPLEX ,   1, (55,255,45), 2)
                        errorx = 0
                        errory = 0
                        if keypoint_scores[0,0] > .04:
                            overlay_image = cv2.line(overlay_image, (centerx,centery - 10), (nosex, nosey), (255, 255, 0), 2)
                            errorx = nosex - centerx
                            errory = nosey - centery - 10
                            if abs(errorx) > 20:
                                ctrl_out_cc = pid_cc(errorx)
                                drone_cc = ctrl_out_cc
                            else:
                                drone_cc = 0
                            if abs(errory) > 16:
                                ctrl_out_ud = pid_ud(errory)
                                drone_ud = ctrl_out_ud
                            else:
                                drone_ud = 0

                            #out_img = cv2.putText(out_img, str(keypoint_scores[ii,kpoint]), (50,50), cv2.FONT_HERSHEY_SIMPLEX ,   1, (255,255,45), 2)
                        else:
                            #reset pid
                            drone_cc = 0
                            drone_ud = 0
                            pid_cc.reset()
                            pid_ud.reset()

                        leftSholy = int(keypoint_coords[0,5,0])
                        rightSholy = int(keypoint_coords[0,6,0])
                        leftHipy = int(keypoint_coords[0,11,0])
                        rightHipy = int(keypoint_coords[0,12,0])
                        meanHeight = ((rightHipy - rightSholy) + (leftHipy - leftSholy))/2 #technically arbitrary
                        desiredHeight = 100
                        ctrl_out_fb = 0

                        errorFB = 0
                        if keypoint_scores[0,5] > .04 and keypoint_scores[0,6] > .04 and keypoint_scores[0,11] > .04 and keypoint_scores[0,12] > .04:
                            errorFB = meanHeight - desiredHeight
                            #error can be within +/- 15 without caring
                            if abs(errorFB) > 15:
                                ctrl_out_fb = pid_cc(errorFB)
                                drone_fb = ctrl_out_fb
                            else:
                                drone_fb = 0
                            #out_img = cv2.putText(out_img, str(keypoint_scores[ii,kpoint]), (50,50), cv2.FONT_HERSHEY_SIMPLEX ,   1, (255,255,45), 2)
                        else:
                            #reset pid
                            drone_fb = 0
                            pid_fb.reset()

                        #don't let the hips lie
                        # if keypoint_scores[0,11] < .04 and keypoint_scores[0,12] < .04:
                        #     drone_fb = -20
                        #     drone_ud = -20

                        #overlay_image = cv2.putText(overlay_image, str(ctrl_out_fb), (30,110), cv2.FONT_HERSHEY_SIMPLEX ,   1, (55,255,45), 2)
                        # overlay_image = cv2.putText(overlay_image, str(ctrl_out_ud), (30,30), cv2.FONT_HERSHEY_SIMPLEX ,   1, (55,255,45), 2)
                        #overlay_image = cv2.putText(overlay_image, str(errory), (30,70), cv2.FONT_HERSHEY_SIMPLEX ,   1, (55,255,45), 2)
                        cv2.imshow('posenet', overlay_image)
                        video.write(overlay_image)
                        #cv2.imshow('Original', image)
                        #cv2.imshow('Canny', cv2.Canny(image, 100, 200))
                        cv2.waitKey(1)
        except KeyboardInterrupt as e:
            print(e)
        except Exception as e:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback)
            print(e)

    cv2.destroyAllWindows()
    video.release()
    drone.quit()
    exit(1)


if __name__ == '__main__':
    main()
