; Auto-generated. Do not edit!


(cl:in-package robot_action_msg-msg)


;//! \htmlinclude robot_acts.msg.html

(cl:defclass <robot_acts> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (gait
    :reader gait
    :initarg :gait
    :type cl:integer
    :initform 0)
   (action
    :reader action
    :initarg :action
    :type cl:integer
    :initform 0))
)

(cl:defclass robot_acts (<robot_acts>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robot_acts>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robot_acts)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_action_msg-msg:<robot_acts> is deprecated: use robot_action_msg-msg:robot_acts instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <robot_acts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_action_msg-msg:type-val is deprecated.  Use robot_action_msg-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'gait-val :lambda-list '(m))
(cl:defmethod gait-val ((m <robot_acts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_action_msg-msg:gait-val is deprecated.  Use robot_action_msg-msg:gait instead.")
  (gait m))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <robot_acts>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_action_msg-msg:action-val is deprecated.  Use robot_action_msg-msg:action instead.")
  (action m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robot_acts>) ostream)
  "Serializes a message object of type '<robot_acts>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'gait)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'action)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robot_acts>) istream)
  "Deserializes a message object of type '<robot_acts>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gait) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'action) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robot_acts>)))
  "Returns string type for a message object of type '<robot_acts>"
  "robot_action_msg/robot_acts")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robot_acts)))
  "Returns string type for a message object of type 'robot_acts"
  "robot_action_msg/robot_acts")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robot_acts>)))
  "Returns md5sum for a message object of type '<robot_acts>"
  "69255ef4e15806c24e79c0d43c19a260")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robot_acts)))
  "Returns md5sum for a message object of type 'robot_acts"
  "69255ef4e15806c24e79c0d43c19a260")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robot_acts>)))
  "Returns full string definition for message of type '<robot_acts>"
  (cl:format cl:nil "int32 type~%int32 gait~%int32 action~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robot_acts)))
  "Returns full string definition for message of type 'robot_acts"
  (cl:format cl:nil "int32 type~%int32 gait~%int32 action~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robot_acts>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robot_acts>))
  "Converts a ROS message object to a list"
  (cl:list 'robot_acts
    (cl:cons ':type (type msg))
    (cl:cons ':gait (gait msg))
    (cl:cons ':action (action msg))
))
