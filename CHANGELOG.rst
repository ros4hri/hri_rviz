^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hri_rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
