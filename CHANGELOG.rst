^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hri_rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-05-21)
------------------
* re-import CHANGELOG from ROS1
* general cleanup
  linting code + fixing wrong dependencies in package.xml
* reading body description from the new dedicated topic
* add LICENSE and CONTRIBUTING.md
* using rviz-ogre-vendor instead of native ogre libraries
  see https://github.com/ros2/rviz/issues/876
* documentation
* fixed icons
* added copyright
* TF (HRI) porting
* Skeletons3D porting
* Humans porting
* Contributors: Séverin Lemaignan, lorenzoferrini

0.4.2 (2023-10-18)
------------------
* moving from non-normalized to normalized facial landmarks
* Contributors: lorenzoferrini

0.4.1 (2023-07-05)
------------------
* changed RoI type to normalized one
* Contributors: Luka Juricic

0.4.0 (2023-01-18)
------------------
* Merge branch 'skeletons' into 'main'
  skeleton extension for Humans plugin
  See merge request ros4hri/hri_rviz!5
* joint visualization as circles
* Optimized iteration over bodies for skeleton representation
* Introducing skeleton visualization
  It is now possible to visualize 2D skeletons using the humans
  rviz plugin.
* add LICENSE
* Contributors: Séverin Lemaignan, lorenzoferrini

0.3.1 (2022-07-12)
------------------
* set license to BSD
* Contributors: Séverin Lemaignan

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

