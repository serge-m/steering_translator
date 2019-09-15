# steering_translator
Steering format translator node for ROS robocar. Converts ML model output to pwm signal.


## Create configuration
```
rosrun rqt_reconfigure rqt_reconfigure
# select steering_translator, set proper pwm values
rosrun dynamic_reconfigure dynparam dump /steering_translator steering_translator.yaml
```

## Load configuration
```
rosrun dynamic_reconfigure dynparam load /steering_translator steering_translator/steering_translator_sample.yaml
```


