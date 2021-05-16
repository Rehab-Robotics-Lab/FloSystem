#!/usr/bin/env python
"""A module to display the robot screen using opencv"""

try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk
import Queue
from PIL import Image, ImageTk
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image as smImage
from cv_bridge import CvBridge, CvBridgeError
from flo_core_defs.srv import SetRecording, SearchGameBucket
from flo_core_defs.msg import GameCommandOptions, GameState, GameCommand, GameDef

# Screen is 800x480

HOME_TIME = 5
UPDATE_HOME_TIME = 1


class PodiumScreen(object):
    """Class to display the podium"""

    # This is a ros node, no need for public methods:
    # pylint: disable=too-few-public-methods

    # Such is the nature of GUIs:
    # pylint: disable=too-many-instance-attributes

    def __init__(self):
        rospy.init_node('podium_screen')

        self.window = tk.Tk()  # Makes main window
        self.window.geometry(
            "{0}x{1}+0+0".format(self.window.winfo_screenwidth(), self.window.winfo_screenheight()))

        main_frame = tk.Frame(self.window)
        main_frame.pack(side=tk.TOP)

        left_frame = tk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT)

        # Video
        vid_frame = tk.Frame(left_frame)
        vid_frame.pack(side=tk.TOP)

        self.displayu = tk.Label(vid_frame)
        self.displayu.grid(row=2, column=0, padx=0, pady=0)  # Display 1

        self.displayl = tk.Label(vid_frame)
        self.displayl.grid(row=1, column=0, padx=0, pady=0)  # Display 1

        self.image_queue_l = Queue.Queue()
        self.image_queue_u = Queue.Queue()

        self.bridge = CvBridge()
        rospy.Subscriber('/upper_realsense/color/image_web',
                         smImage, self.__new_img_l)
        rospy.Subscriber('/lower_realsense/color/image_web',
                         smImage, self.__new_img_u)

        # Recording
        rec_frame = tk.Frame(left_frame)
        rec_frame.pack(side=tk.TOP)

        self.record_text = tk.Label(rec_frame)
        self.record_text.grid(row=3, column=0)

        rospy.wait_for_service('set_recording')
        self.record_button = tk.Button(
            rec_frame, text="Start Recoring", command=self.__start_recording)
        self.record_button.grid(row=4, column=0)

        self.record_button = tk.Button(
            rec_frame, text="Stop Recoring", command=self.__stop_recording)
        self.record_button.grid(row=4, column=0)

        self.record_button = tk.Button(
            rec_frame, text="Start Recoring", command=self.__start_recording)
        self.record_button.grid(row=5, column=0)

        self.recording = False

        rospy.Subscriber('/record_video_status', Bool,
                         self.__set_recording_state)

        # Game Runner
        game_frame = tk.Frame(main_frame)
        game_frame.pack(side=tk.LEFT)

        self.game_status_l = tk.Label(game_frame, text='unknown')
        self.game_status_l.pack(side=tk.TOP)

        game_def_frame = tk.Frame(game_frame)
        game_def_frame.pack(side=tk.TOP)

        rospy.wait_for_service('search_game_bucket_name_desc')
        search_gb = rospy.ServiceProxy(
            'search_game_bucket_name_desc', SearchGameBucket)
        self.game_buckets = search_gb('').game_buckets

        self.game_opts_frame = tk.Frame(game_frame)
        self.game_opts_frame.pack(side=tk.TOP)

        self.game_opts = []
        self.updated_game_opts = True
        self.game_command_pub = rospy.Publisher(
            '/game_runner_commands', GameCommand, queue_size=1)
        rospy.Subscriber('/game_runner_command_opts',
                         GameCommandOptions, self.__set_game_opts)

        self.game_state = ''
        rospy.Subscriber('/game_runner_state',
                         GameState, self.__set_game_state)

        self.simon_says_b = tk.Button(
            game_def_frame, text="Start Simon Says", command=self.__start_simon_says)
        self.simon_says_b.grid(row=1, column=1)

        self.target_touch_b = tk.Button(
            game_def_frame, text="Start Target Touch", command=self.__start_target_touch)
        self.target_touch_b.grid(row=2, column=1)

        game_text_frame = tk.Frame(self.window)
        game_text_frame.pack(side=tk.TOP, fill=tk.X)
        self.game_text_label = tk.Label(game_text_frame)
        self.game_text_label.configure(
            wraplength=500, font=('Arial', 50))
        self.game_text_label.bind('<Configure>', lambda e: self.game_text_label.config(
            wraplength=self.game_text_label.winfo_width()))
        self.game_text_label.pack(side=tk.TOP, fill=tk.X)

        self.game_text = ''
        rospy.Subscriber('/game_runner_text', String, self.__set_game_text)

        rospy.loginfo('Started Podium Screen Node')

        self.__run_display()

    def __set_recording_state(self, msg):
        self.recording = msg.data

    def __run_display(self):
        rate = rospy.Rate(45)
        while not rospy.is_shutdown():
            # update images
            img_l = None
            img_u = None
            empty = False
            while not empty:
                try:
                    img_l = self.image_queue_l.get_nowait()
                except Queue.Empty:
                    empty = True
            empty = False
            while not empty:
                try:
                    img_u = self.image_queue_u.get_nowait()
                except Queue.Empty:
                    empty = True
            if img_l is not None:
                img = Image.fromarray(img_l)
                imgtk = ImageTk.PhotoImage(master=self.displayl, image=img)
                self.displayl.imgtk = imgtk
                self.displayl.configure(image=imgtk)
            if img_u is not None:
                img = Image.fromarray(img_u)
                imgtk = ImageTk.PhotoImage(master=self.displayu, image=img)
                self.displayu.imgtk = imgtk
                self.displayu.configure(image=imgtk)

            # update recording
            if self.recording:
                self.record_text.configure(text='Recording', fg='green')
            else:
                self.record_text.configure(text='Not Recording', fg='red')

            # update game state
            self.game_status_l.configure(text=self.game_state)
            if self.game_state == 'waiting_for_def':
                self.simon_says_b.configure(state=tk.NORMAL)
                self.target_touch_b.configure(state=tk.NORMAL)
            else:
                self.simon_says_b.configure(state=tk.DISABLED)
                self.target_touch_b.configure(state=tk.DISABLED)

            # Update game options
            if self.updated_game_opts:
                for widget in self.game_opts_frame.winfo_children():
                    widget.destroy()
                for opt in self.game_opts:
                    t_button = tk.Button(self.game_opts_frame, text=opt,
                                         command=lambda opt=opt: self.__send_command(opt))
                    t_button.pack(side=tk.TOP)
                self.updated_game_opts = False

            self.game_text_label.configure(text=self.game_text)

            self.window.update_idletasks()
            self.window.update()

            rate.sleep()

    def __new_img_l(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as err:
            rospy.logerr('error converting message to cvmat: %s', err)
            return
        # res_img = cv2.resize(cv_image, (800, 480))
        self.image_queue_l.put(cv_image)

    def __new_img_u(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as err:
            rospy.logerr('error converting message to cvmat: %s', err)
            return
        # res_img = cv2.resize(cv_image, (800, 480))
        self.image_queue_u.put(cv_image)

    @staticmethod
    def __set_recording(val):
        service_proxy = rospy.ServiceProxy('set_recording', SetRecording)
        try:
            service_proxy(val)
        except rospy.ServiceException as exc:
            tk.messagebox.showwarning(
                'Recording', 'Unable to set recording state: {}'.format(exc))

    def __start_recording(self):
        self.__set_recording(True)

    def __stop_recording(self):
        self.__set_recording(False)

    def __set_game_opts(self, msg):
        self.game_opts = msg.options
        self.updated_game_opts = True

    def __send_command(self, opt):
        self.game_command_pub.publish(opt)

    def __set_game_state(self, msg):
        self.game_state = msg.state

    @staticmethod
    def __start_game(game_def):
        pub = rospy.Publisher('game_runner_def',
                              GameDef, queue_size=0, latch=True)
        pub.publish(game_def)

    def __start_simon_says(self):
        game_def = GameDef()
        game_def.game_type = 'simon_says'
        game_def.steps = self.game_buckets[0].steps
        game_def.bimanual = True
        self.__start_game(game_def)

    def __start_target_touch(self):
        game_def = GameDef()
        game_def.game_type = 'target_touch'
        game_def.steps = self.game_buckets[1].steps
        game_def.reps = 3
        game_def.min_steps = 2
        game_def.max_steps = 4
        game_def.bimanual = True
        self.__start_game(game_def)

    def __set_game_text(self, msg):
        self.game_text = msg.data


if __name__ == '__main__':
    PodiumScreen()
