; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude scan_range.msg.html

(cl:defclass <scan_range> (roslisp-msg-protocol:ros-message)
  ((angle_min
    :reader angle_min
    :initarg :angle_min
    :type cl:float
    :initform 0.0)
   (angle_max
    :reader angle_max
    :initarg :angle_max
    :type cl:float
    :initform 0.0)
   (range_min
    :reader range_min
    :initarg :range_min
    :type cl:float
    :initform 0.0)
   (range_max
    :reader range_max
    :initarg :range_max
    :type cl:float
    :initform 0.0))
)

(cl:defclass scan_range (<scan_range>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <scan_range>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'scan_range)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<scan_range> is deprecated: use beginner_tutorials-msg:scan_range instead.")))

(cl:ensure-generic-function 'angle_min-val :lambda-list '(m))
(cl:defmethod angle_min-val ((m <scan_range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:angle_min-val is deprecated.  Use beginner_tutorials-msg:angle_min instead.")
  (angle_min m))

(cl:ensure-generic-function 'angle_max-val :lambda-list '(m))
(cl:defmethod angle_max-val ((m <scan_range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:angle_max-val is deprecated.  Use beginner_tutorials-msg:angle_max instead.")
  (angle_max m))

(cl:ensure-generic-function 'range_min-val :lambda-list '(m))
(cl:defmethod range_min-val ((m <scan_range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:range_min-val is deprecated.  Use beginner_tutorials-msg:range_min instead.")
  (range_min m))

(cl:ensure-generic-function 'range_max-val :lambda-list '(m))
(cl:defmethod range_max-val ((m <scan_range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:range_max-val is deprecated.  Use beginner_tutorials-msg:range_max instead.")
  (range_max m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <scan_range>) ostream)
  "Serializes a message object of type '<scan_range>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <scan_range>) istream)
  "Deserializes a message object of type '<scan_range>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range_max) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<scan_range>)))
  "Returns string type for a message object of type '<scan_range>"
  "beginner_tutorials/scan_range")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'scan_range)))
  "Returns string type for a message object of type 'scan_range"
  "beginner_tutorials/scan_range")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<scan_range>)))
  "Returns md5sum for a message object of type '<scan_range>"
  "9231e3ac3c9a27ca200edbb34ea8ab8d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'scan_range)))
  "Returns md5sum for a message object of type 'scan_range"
  "9231e3ac3c9a27ca200edbb34ea8ab8d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<scan_range>)))
  "Returns full string definition for message of type '<scan_range>"
  (cl:format cl:nil "float32 angle_min        # start angle of the scan [rad]~%float32 angle_max        # end angle of the scan [rad]~%~%float32 range_min        # minimum range value [m]~%float32 range_max		 # maximum range value [m]~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'scan_range)))
  "Returns full string definition for message of type 'scan_range"
  (cl:format cl:nil "float32 angle_min        # start angle of the scan [rad]~%float32 angle_max        # end angle of the scan [rad]~%~%float32 range_min        # minimum range value [m]~%float32 range_max		 # maximum range value [m]~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <scan_range>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <scan_range>))
  "Converts a ROS message object to a list"
  (cl:list 'scan_range
    (cl:cons ':angle_min (angle_min msg))
    (cl:cons ':angle_max (angle_max msg))
    (cl:cons ':range_min (range_min msg))
    (cl:cons ':range_max (range_max msg))
))
