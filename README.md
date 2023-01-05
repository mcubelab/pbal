# pbal
This repository implements the ["contact configuration regulation"](https://arxiv.org/abs/2203.01203) estimator and controller developed by Orion Taylor, Neel Doshi, and Alberto Rodriguez.

# stuff you'll need to install if you want to get this to run
-To get any of this to run, you'll need python 3, I'm not sure which version. Some of the scripts will run just using python 2.7 but any of the estimators that use gtsam need some flavor of python 3 to run
-[gtsam](https://github.com/borglab/gtsam), (with the python bindings) is needed to run any of the estimation scripts that use gtsam (usually denoted by gtsam_ in the filename)


# how to run
I've done my best to set the scripts up so that they are capable of running on test data without actually being connected to a robot. 
[link to dropbox with experiment data:](https://www.dropbox.com/sh/8ot92g1z51pha5u/AADT0IPdf4_IEFz9unZ31ERsa?dl=0)

If you want to run one of the ROS node scripts on the test data (and without ROS) find the line that says the following: 

```
  rm = ros_manager()
```

This creates an instance of the ros_manager class, which does all the dirty work of creating the ros node, spawning publishers/subscribers and all that other junk that would make the estimator/controller scripts unreadable. The ros_manager class has two mode: default and load. In the default mode, it actually does all that ROS junk, and so you would actually need ROS, and a bunch of other extra stuff specific to our robot platform in the lab to get it to run properly. If you aren't named Orion or Neel, you probably shouldn't be running this in default mode. In load mode, ros_manager loads a .pickle and .avi (which store all the ros messages for a single experiment) so that you can run the script using the experimental data. 


```
  #use if playing from pickle file
  path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
  # path = '/home/taylorott/Documents/experiment_data/gtsam_test_data_fall_2022'
  fname = '/test_data-experiment0024.pickle'
  rm = ros_manager(load_mode = True, path=path, fname=fname)

```

