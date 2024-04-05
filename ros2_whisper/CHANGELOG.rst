^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ROS 2 Whisper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.2.1 (2023-12-15)
------------------
* `whisper_cpp_vendor`: Set C++ standard to C++11 for target

1.2.0 (2023-11-19)
------------------
* `whisper_util`: Upgrade to `whisper.cpp` 1.5.0 release https://github.com/ggerganov/whisper.cpp/releases/tag/v1.5.0 (full CUDA backend)

1.1.0 (2023-09-01)
------------------
* `whisper_demos`: Improved terminal output
* `whisper_server`: Improved state machine

1.0.0 (2023-08-31)
------------------
* Initial release
* Uses whisper.cpp https://github.com/ggerganov/whisper.cpp instead of OpenAI Whisper Python library
* Introduces `whisper_bringup` package for launch
* Introduces `whisper_cpp_vendor` to expose whisper.cpp as a ROS 2 package
* Introduces `whisper_demos` for exemplary usage of `whisper_server`
* Introduces `whisper_msgs` for ROS 2 message definitions
* Introduces `whisper_server` for whisper action server
* Introduces `whisper_utils` package for inference utilities

alpha 0.1.0 (2023-08-15)
------------------------
* Alpha release
* Uses OpenAI Whisper Python library https://github.com/openai/whisper
