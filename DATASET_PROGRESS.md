# Progress in the dataset to train our model

*Nav2 Goals/routes:*

- **rotundan**         - (ready for collection)
- **road_to_west**     - (ready for collection)
- **parkinglot_west**  - **_(COLLECTED)_**
- **road_to_spawn**    - (ready for collection)
- **spawn**            - (ready for collection)
- **road_to_east**     - (ready for collection)
- **parkinglot_east**  - (ready for collection)
- **road_to_strip**    - (ready for collection)
- **strip**            - *(needs tuning)*
- **art**              - *(needs tuning)*
- **playground**       - *(needs tuning)*

## ISSUES

* When running with *spawn_uav:=false* all **dji0** topics are still published at 10 Hz.
* */a201_0000/bond* publishes at 200 Hz.
* */coord* and other topics are published by camera_tracker node even when we don't spawn a UAV for following.