# pbal
This repository implements the ["contact configuration regulation"](https://arxiv.org/abs/2203.01203) estimator and controller developed by Orion Taylor, Neel Doshi, and Alberto Rodriguez.

# stuff you'll need to install if you want to get this to run
-To get any of this to run, you'll need python 3, I'm not sure which version. Some of the scripts will run just using python 2.7 but any of the estimators that use gtsam need some flavor of python 3 to run
-[gtsam](https://github.com/borglab/gtsam), (with the python bindings) is needed to run any of the estimation scripts that use gtsam (usually denoted by gtsam_ in the filename)

I've done my best to set the scripts up so that they are capable of running on test data without actually being connected to a robot. 
[link to dropbox with experiment data:](https://www.dropbox.com/sh/8ot92g1z51pha5u/AADT0IPdf4_IEFz9unZ31ERsa?dl=0)

```
  #use if playing from pickle file
  path = '/home/thecube/Documents/pbal_experiments/gtsam_test_data_fall_2022'
  # path = '/home/taylorott/Documents/experiment_data/gtsam_test_data_fall_2022'
  fname = '/test_data-experiment0024.pickle'
  rm = ros_manager(load_mode = True, path=path, fname=fname)

```

