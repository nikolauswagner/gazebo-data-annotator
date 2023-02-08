# gazebo-data-annotator
Data annotation tool for Gazebo-based, virtually created training images

Gazebo simulation:

![Gazebo simulation](docs/gazebo.png)

Resulting annotations:

![Annotations](docs/annotated.png)

To use this tool, run the [training data generation script](https://github.com/Melanoneiro/gazebo-data-annotator/blob/master/src/gazebo-data-annotator/training_data_generator.py), which (at the moment) uses a hardcoded keyword to search for (e.g. "strawberry"). It will then annotate all gazebo objects in the viewing field of the camera (make sure to change the name of the camera reference frame and, if needed, the camera topics) that contain the keyword.

This tool relies on the [Gazebo Segmentation Plugin](https://github.com/Melanoneiro/gazebo_segmentation) for obtaining object IDs and such.
