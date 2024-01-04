^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cpr_indoornav_base
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2024-01-04)
------------------
* Create common installation classes to reduce code duplication in dependent packages
* Update base package to use Otto's 2.26 release
* Contributors: Chris Iverach-Brereton

0.3.4 (2022-11-24)
------------------
* Update docking controller file
* Add wireless docking support
* Assume always 2 lasers in indoornav
* Fix value to default in launch
* Moved controller launch to each repo since envvars relative to each robot
* Added lanuch files for running indoornav docking
* Add instructions for building the ROS2 SDK examples since they require manual fixes
* Contributors: Chris Iverach-Brereton, José Mastrangelo

0.3.3 (2022-06-03)
------------------
* Update the ROS2 domain IDs since they've changed on the latest IndoorNav ISO
* Contributors: Chris Iverach-Brereton

0.3.2 (2022-05-13)
------------------
* Use python3 not python for the tf republisher script
* Contributors: Chris Iverach-Brereton

0.3.1 (2022-05-12)
------------------
* Add additional relays to expose IndoorNav backpack topics to external hosts
* Add a simple tf republisher to ensure the tf between map and odom is visible
  externally
* Contributors: Chris Iverach-Brereton

0.3.0 (2022-05-06)
------------------
* Initial public release

0.2.0 (2022-02-22)
------------------
* Merge branch 'melodic-devel' into noetic-devel
* Initial release for Noetic
* Contributors: Chris Iverach-Brereton

0.1.1 (2022-02-22)
------------------
* Refactor the package structure, rename packages for clarity
* Contributors: Chris Iverach-Brereton

0.1.0 (2022-02-16)
------------------
* Inital Release
* Contributors: Chris Iverach-Brereton
