^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hri_rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2022-07-12)
------------------
* Added PAL Robotics copyright
* Refactored skeleton frames testing
  The new version takes a frame name, verifies if it has more than
  the minimum number of characters for a skeleton frame, checks
  if the name without the last 5 characters belongs to the skeleton
  ROS4HRI naming convention and if it belongs to a currently tracked
  body.
* bodies -> skeletons
  Bodies frames are now referenced as skeleton frame, for clarity.
* Redefined class name for the plugin
  To avoid confusion with the already existing TF plugin, the
  plugin will now be visualized as TF (HRI) in rviz
* First complete hri_tf version
* Fixed skeletons appearing when plugin was disabled
* Contributors: Séverin Lemaignan, lorenzoferrini

0.2.0 (2022-03-06)
------------------
* Facial landmarks visualization
* raw pointers to std::share_ptr
* add basic README
* Contributors: Séverin Lemaignan, lorenzoferrini

0.1.3 (2022-01-21)
------------------
* update to libhri 0.2.3
* Plugin renaming: Faces --> Humans
* Contributors: Séverin Lemaignan, lorenzoferrini

0.1.2 (2022-01-14)
------------------
* replace hri_msgs::RegionOfInterestStamped by sensor_msgs::RegionOfInterest
  Follows changes in hri_msgs 0.2.0
* Contributors: Séverin Lemaignan

0.1.1 (2022-01-13)
------------------
* remove code that relies on too-recent rviz APIs, to ensure compat with melodic/noetic
* code formatting + PAL copyright
* add missing dependencies + fix flags
* Contributors: Séverin Lemaignan

0.1.0 (2022-01-13)
------------------

* port the plugin to libhri; use colors generated from ID for bbs
* Multi-Skeleton 3D visualization plugin
* Faces Plugin as ImageDisplay-like object
  Faces Plugin displaying multiple faces simultaneously, with
  random colors for each bounding box available. Currently,
  we select a random color directly from the RGB color model. In
  the future, we will select a color from the HSB model, fixing
  S and B values to get sufficiently bright and vibrant colors.
* Contributors: Séverin Lemaignan, Lorenzo Ferrini
