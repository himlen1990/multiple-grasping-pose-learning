#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "multiple_grasping_pose_learning")
(ros::roseus "call_back_test")

(format t "init")
(ros::wait-for-service "aero_goods_grasping_demo")

(defun setup()
  (load "package://aeroeus/aero-interface.l")
  (aero-init)
  (setq *robot* *aero*)
  (objects (list *robot*))
  )


(defun get_obj_pose()
  (let (res
	get-obj-pose-srv
	obj-pose-res
	pose)
    
    (setq get-obj-pose-srv (instance multiple_grasping_pose_learning::getobjectposeRequest :init
				    :command "test"))
    (setq obj-pose-res (ros::service-call
			"aero_goods_grasping_demo"
			get-obj-pose-srv))
    (setq pose (send obj-pose-res :GraspPose))
    (format t "~A~%" pose)
    pose
    ))

(defun pregrasp()
  (setq pose (get_obj_pose))
  (setq *co* (make-coords))
  (send *co* :translate pose)
  (send *co* :translate (float-vector -60 0 80))
  (send *robot* :inverse-kinematics
	*co*
	:ik-group :rarm-upper-body :rotation-axis t
	:revert-if-fail nil :look-at-target t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
  (objects (list *co* *robot*))
  )

(defun grasp()
  (send *co* :translate (float-vector 130 30 0))
  (send *robot* :inverse-kinematics
	*co*
	:ik-group :rarm-upper-body :rotation-axis t
	:revert-if-fail nil :look-at-target t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
  (unix:usleep(* 6000 1000))
  (send *ri* :start-grasp :rarm)
  (objects (list *co* *robot*))
  )

(defun rotation()
  (let (pre_release_pose)
    (send *co* :rotate (deg2rad 30) :z :world)
    (send *co* :translate (float-vector 0 -20 0) :world);; bias compensate ;; 20 -20 0
    (send *robot* :inverse-kinematics
	  *co*
	  :ik-group :rarm-upper-body :rotation-axis t
	  :revert-if-fail nil :look-at-target t)
    (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
    (unix:usleep(* 6000 1000))
    (send *ri* :stop-grasp :rarm)
    (send *co* :translate (float-vector -50 0 0) :local)
    (send *robot* :inverse-kinematics
	  *co*
	  :ik-group :rarm-upper-body :rotation-axis t
	  :revert-if-fail nil :look-at-target t)
    (objects (list *co* *robot*))
    (setq pre_release_pose (send *robot* :angle-vector))
    (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
    (unix:usleep(* 8000 1000))
    (send *robot* :reset-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
    pre_release_pose
    ))

(defun collect_data_once()
  (pregrasp)
  (unix:usleep(* 8000 1000))
  (grasp)
  (unix:usleep(* 3000 1000))
  (setq pre_release_pose (rotation))
  (unix:usleep(* 8000 1000))
  (get_obj_pose) ;;send req to save images
  
  (send *robot* :angle-vector pre_release_pose)
  (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
  (unix:usleep(* 6000 1000))
  (send *co* :translate (float-vector 50 0 0) :local)
  (send *robot* :inverse-kinematics
	*co*
	:ik-group :rarm-upper-body :rotation-axis t
	:revert-if-fail nil :look-at-target t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
  (unix:usleep(* 6000 1000))
  (send *ri* :start-grasp :rarm)  
  (setq pre_release_pose (rotation))
  )

(defun collect_data()
  (collect_data_once)
  (unix:usleep(* 6000 1000))
  (collect_data_once)
)

(setup)
;;(collect_data_once)
;;(unix:usleep(* 6000 1000))
;;(collect_data_once)
(send *robot* :reset-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 6000)
