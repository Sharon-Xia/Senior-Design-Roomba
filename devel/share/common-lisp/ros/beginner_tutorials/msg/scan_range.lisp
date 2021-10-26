; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude scan_range.msg.html

(cl:defclass <scan_range> (roslisp-msg-protocol:ros-message)
  ((closest_point
    :reader closest_point
    :initarg :closest_point
    :type cl:float
    :initform 0.0)
   (farthest_point
    :reader farthest_point
    :initarg :farthest_point
    :type cl:float
    :initform 0.0))
)

(cl:defclass scan_range (<scan_range>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <scan_range>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'scan_range)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<scan_range> is deprecated: use beginner_tutorials-msg:scan_range instead.")))

(cl:ensure-generic-function 'closest_point-val :lambda-list '(m))
(cl:defmethod closest_point-val ((m <scan_range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:closest_point-val is deprecated.  Use beginner_tutorials-msg:closest_point instead.")
  (closest_point m))

(cl:ensure-generic-function 'farthest_point-val :lambda-list '(m))
(cl:defmethod farthest_point-val ((m <scan_range>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:farthest_point-val is deprecated.  Use beginner_tutorials-msg:farthest_point instead.")
  (farthest_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <scan_range>) ostream)
  "Serializes a message object of type '<scan_range>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'closest_point))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'farthest_point))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <scan_range>) istream)
  "Deserializes a message object of type '<scan_range>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'closest_point) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'farthest_point) (roslisp-utils:decode-double-float-bits bits)))
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
  "2d10e7530caaa326bc93d51700c285d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'scan_range)))
  "Returns md5sum for a message object of type 'scan_range"
  "2d10e7530caaa326bc93d51700c285d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<scan_range>)))
  "Returns full string definition for message of type '<scan_range>"
  (cl:format cl:nil "float64 closest_point        # minimum range value [m]~%float64 farthest_point		 # maximum range value [m]~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'scan_range)))
  "Returns full string definition for message of type 'scan_range"
  (cl:format cl:nil "float64 closest_point        # minimum range value [m]~%float64 farthest_point		 # maximum range value [m]~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <scan_range>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <scan_range>))
  "Converts a ROS message object to a list"
  (cl:list 'scan_range
    (cl:cons ':closest_point (closest_point msg))
    (cl:cons ':farthest_point (farthest_point msg))
))
