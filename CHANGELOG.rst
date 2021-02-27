^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package layered_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.9 (2021-02-28)
------------------
* c++11
* Destruct layer instances before unloading layer plugins

0.0.8 (2020-05-05)
------------------
* Refactor LayeredHardware to be used as a parent class

0.0.7 (2020-05-03)
------------------
* Reset position-based joint limits when switching controllers

0.0.6 (2020-04-28)
------------------
* Add accessor methods to layers (LayeredHardware::{size(), layer()})
* Add link to layered_hardware_gazebo package in README

0.0.5 (2020-02-28)
------------------
* Enable a link to layered_hardware_epos in README
* Make robot description optional

0.0.4 (2019-12-25)
------------------
* Support soft joint limits

0.0.3 (2019-12-25)
------------------
* Updated example
* support LayerBase::prepareSwitch()

0.0.2 (2019-12-24)
------------------
* Refactored layers
* Updated README

0.0.1 (2019-12-13)
------------------
* Initial version
* Compilable on kinetic & melodic
* Tested with launch/example.launch
* Contributors: Yoshito Okada
