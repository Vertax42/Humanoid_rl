;; Auto-generated. Do not edit!


(when (boundp 'robot_action_msg::robot_acts)
  (if (not (find-package "ROBOT_ACTION_MSG"))
    (make-package "ROBOT_ACTION_MSG"))
  (shadow 'robot_acts (find-package "ROBOT_ACTION_MSG")))
(unless (find-package "ROBOT_ACTION_MSG::ROBOT_ACTS")
  (make-package "ROBOT_ACTION_MSG::ROBOT_ACTS"))

(in-package "ROS")
;;//! \htmlinclude robot_acts.msg.html


(defclass robot_action_msg::robot_acts
  :super ros::object
  :slots (_type _gait _action ))

(defmethod robot_action_msg::robot_acts
  (:init
   (&key
    ((:type __type) 0)
    ((:gait __gait) 0)
    ((:action __action) 0)
    )
   (send-super :init)
   (setq _type (round __type))
   (setq _gait (round __gait))
   (setq _action (round __action))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:gait
   (&optional __gait)
   (if __gait (setq _gait __gait)) _gait)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:serialization-length
   ()
   (+
    ;; int32 _type
    4
    ;; int32 _gait
    4
    ;; int32 _action
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _type
       (write-long _type s)
     ;; int32 _gait
       (write-long _gait s)
     ;; int32 _action
       (write-long _action s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _gait
     (setq _gait (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _action
     (setq _action (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robot_action_msg::robot_acts :md5sum-) "69255ef4e15806c24e79c0d43c19a260")
(setf (get robot_action_msg::robot_acts :datatype-) "robot_action_msg/robot_acts")
(setf (get robot_action_msg::robot_acts :definition-)
      "int32 type
int32 gait
int32 action
")



(provide :robot_action_msg/robot_acts "69255ef4e15806c24e79c0d43c19a260")


