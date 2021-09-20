#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "multiple_grasping_pose_learning")
(ros::roseus "execution")

(format t "init")
(ros::wait-for-service "aero_goods_grasping_deploy")

(defun setup()
  (load "package://aeroeus/aero-interface.l")
  (aero-init)
  (setq *robot* *aero*)
  (objects (list *robot*))
  )


(defun get_object_pose(obj_class)
  (let (res
	get-object-pose-srv
	object-pose-res
	pose)
    
    (setq get-object-pose-srv (instance multiple_grasping_pose_learning::graspanobjectRequest :init
				     :TargetObj obj_class
				     :WorkSpace (float-vector 0.6 -0.3 0.95 0.88 0.2 1.2))) ;;default workspace (xmin, ymin, zmin, xmax, ymax, zmax)
    (setq object-pose-res (ros::service-call
			"aero_goods_grasping_deploy"
			get-object-pose-srv))
    (setq pose (send object-pose-res :Pose))    
    (format t "~A~%" pose)
    pose
    ))


(defun grasp_object(obj_class)
  (setq *object_pose* (get_object_pose obj_class))
  (setq obj-pos-pregrasp (scale 1000 (subseq *object_pose* 0 3)));;get object position, multiply 1000 for m to mm
  (setq obj-rot-pregrasp (quaternion2matrix (subseq *object_pose* 3 7)));;get object rotation
  (setq obj-pos-grasp (scale 1000 (subseq *object_pose* 7 10)))
  (setq obj-rot-grasp (quaternion2matrix (subseq *object_pose* 10 14)));;get object rotation
  ;;pregrasp
  (setq *pregrasp* (make-coords :pos obj-pos-pregrasp :rot obj-rot-pregrasp))
					;(send *pregrasp* :translate (float-vector 0 0 50) :world)
  (send *robot* :inverse-kinematics
	*pregrasp*
	:ik-group :rarm-whole-body :rotation-axis t
	:revert-if-fail nil :look-at-target t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
  (unix:usleep(* 6000 1000))
  ;;grasp
  (send *pregrasp* :translate (float-vector 0 0 10) :world)
  (setq *grasp* (make-coords :pos obj-pos-grasp :rot obj-rot-grasp))
  (send *robot* :inverse-kinematics
	*grasp*
	:ik-group :rarm-whole-body :rotation-axis t
	:revert-if-fail nil :look-at-target t)
  (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
  (unix:usleep(* 6000 1000))
  (send *ri* :start-grasp :rarm)
  (unix:usleep(* 3000 1000))
  (send *robot* :reset-pose)
  (send *ri* :angle-vector (send *robot* :angle-vector) 6000)
  (unix:usleep(* 6000 1000))
  (send *ri* :stop-grasp :rarm)
  )

(setup)
(send *robot* :reset-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 6000)
;;(execution)
