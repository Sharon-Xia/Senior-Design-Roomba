;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::scan_range)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'scan_range (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::SCAN_RANGE")
  (make-package "BEGINNER_TUTORIALS::SCAN_RANGE"))

(in-package "ROS")
;;//! \htmlinclude scan_range.msg.html


(defclass beginner_tutorials::scan_range
  :super ros::object
  :slots (_angle_min _angle_max _range_min _range_max ))

(defmethod beginner_tutorials::scan_range
  (:init
   (&key
    ((:angle_min __angle_min) 0.0)
    ((:angle_max __angle_max) 0.0)
    ((:range_min __range_min) 0.0)
    ((:range_max __range_max) 0.0)
    )
   (send-super :init)
   (setq _angle_min (float __angle_min))
   (setq _angle_max (float __angle_max))
   (setq _range_min (float __range_min))
   (setq _range_max (float __range_max))
   self)
  (:angle_min
   (&optional __angle_min)
   (if __angle_min (setq _angle_min __angle_min)) _angle_min)
  (:angle_max
   (&optional __angle_max)
   (if __angle_max (setq _angle_max __angle_max)) _angle_max)
  (:range_min
   (&optional __range_min)
   (if __range_min (setq _range_min __range_min)) _range_min)
  (:range_max
   (&optional __range_max)
   (if __range_max (setq _range_max __range_max)) _range_max)
  (:serialization-length
   ()
   (+
    ;; float32 _angle_min
    4
    ;; float32 _angle_max
    4
    ;; float32 _range_min
    4
    ;; float32 _range_max
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle_min
       (sys::poke _angle_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_max
       (sys::poke _angle_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _range_min
       (sys::poke _range_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _range_max
       (sys::poke _range_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle_min
     (setq _angle_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_max
     (setq _angle_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _range_min
     (setq _range_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _range_max
     (setq _range_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get beginner_tutorials::scan_range :md5sum-) "9231e3ac3c9a27ca200edbb34ea8ab8d")
(setf (get beginner_tutorials::scan_range :datatype-) "beginner_tutorials/scan_range")
(setf (get beginner_tutorials::scan_range :definition-)
      "float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]

float32 range_min        # minimum range value [m]
float32 range_max		 # maximum range value [m]
")



(provide :beginner_tutorials/scan_range "9231e3ac3c9a27ca200edbb34ea8ab8d")


