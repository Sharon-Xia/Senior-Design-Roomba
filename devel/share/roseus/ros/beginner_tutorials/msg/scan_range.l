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
  :slots (_closest_point _farthest_point ))

(defmethod beginner_tutorials::scan_range
  (:init
   (&key
    ((:closest_point __closest_point) 0.0)
    ((:farthest_point __farthest_point) 0.0)
    )
   (send-super :init)
   (setq _closest_point (float __closest_point))
   (setq _farthest_point (float __farthest_point))
   self)
  (:closest_point
   (&optional __closest_point)
   (if __closest_point (setq _closest_point __closest_point)) _closest_point)
  (:farthest_point
   (&optional __farthest_point)
   (if __farthest_point (setq _farthest_point __farthest_point)) _farthest_point)
  (:serialization-length
   ()
   (+
    ;; float64 _closest_point
    8
    ;; float64 _farthest_point
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _closest_point
       (sys::poke _closest_point (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _farthest_point
       (sys::poke _farthest_point (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _closest_point
     (setq _closest_point (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _farthest_point
     (setq _farthest_point (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get beginner_tutorials::scan_range :md5sum-) "2d10e7530caaa326bc93d51700c285d9")
(setf (get beginner_tutorials::scan_range :datatype-) "beginner_tutorials/scan_range")
(setf (get beginner_tutorials::scan_range :definition-)
      "float64 closest_point        # minimum range value [m]
float64 farthest_point		 # maximum range value [m]
")



(provide :beginner_tutorials/scan_range "2d10e7530caaa326bc93d51700c285d9")


