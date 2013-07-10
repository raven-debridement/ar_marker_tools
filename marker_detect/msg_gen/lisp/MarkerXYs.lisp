; Auto-generated. Do not edit!


(cl:in-package marker_detect-msg)


;//! \htmlinclude MarkerXYs.msg.html

(cl:defclass <MarkerXYs> (roslisp-msg-protocol:ros-message)
  ((ids
    :reader ids
    :initarg :ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (xys
    :reader xys
    :initarg :xys
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (oris
    :reader oris
    :initarg :oris
    :type (cl:vector geometry_msgs-msg:Quaternion)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Quaternion :initial-element (cl:make-instance 'geometry_msgs-msg:Quaternion))))
)

(cl:defclass MarkerXYs (<MarkerXYs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MarkerXYs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MarkerXYs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name marker_detect-msg:<MarkerXYs> is deprecated: use marker_detect-msg:MarkerXYs instead.")))

(cl:ensure-generic-function 'ids-val :lambda-list '(m))
(cl:defmethod ids-val ((m <MarkerXYs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marker_detect-msg:ids-val is deprecated.  Use marker_detect-msg:ids instead.")
  (ids m))

(cl:ensure-generic-function 'xys-val :lambda-list '(m))
(cl:defmethod xys-val ((m <MarkerXYs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marker_detect-msg:xys-val is deprecated.  Use marker_detect-msg:xys instead.")
  (xys m))

(cl:ensure-generic-function 'oris-val :lambda-list '(m))
(cl:defmethod oris-val ((m <MarkerXYs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader marker_detect-msg:oris-val is deprecated.  Use marker_detect-msg:oris instead.")
  (oris m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MarkerXYs>) ostream)
  "Serializes a message object of type '<MarkerXYs>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'xys))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'xys))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'oris))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'oris))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MarkerXYs>) istream)
  "Deserializes a message object of type '<MarkerXYs>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'xys) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'xys)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'oris) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'oris)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Quaternion))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MarkerXYs>)))
  "Returns string type for a message object of type '<MarkerXYs>"
  "marker_detect/MarkerXYs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MarkerXYs)))
  "Returns string type for a message object of type 'MarkerXYs"
  "marker_detect/MarkerXYs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MarkerXYs>)))
  "Returns md5sum for a message object of type '<MarkerXYs>"
  "c07f000d1086329eaac495a4e5e32e3c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MarkerXYs)))
  "Returns md5sum for a message object of type 'MarkerXYs"
  "c07f000d1086329eaac495a4e5e32e3c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MarkerXYs>)))
  "Returns full string definition for message of type '<MarkerXYs>"
  (cl:format cl:nil "uint32[] ids~%geometry_msgs/Point[] xys~%geometry_msgs/Quaternion[] oris~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MarkerXYs)))
  "Returns full string definition for message of type 'MarkerXYs"
  (cl:format cl:nil "uint32[] ids~%geometry_msgs/Point[] xys~%geometry_msgs/Quaternion[] oris~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MarkerXYs>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'xys) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'oris) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MarkerXYs>))
  "Converts a ROS message object to a list"
  (cl:list 'MarkerXYs
    (cl:cons ':ids (ids msg))
    (cl:cons ':xys (xys msg))
    (cl:cons ':oris (oris msg))
))
