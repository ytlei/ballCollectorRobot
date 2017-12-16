#!/bin/bash

# add ball coord to the world
rosservice call /add_target '[1, 0, 0]'
rosservice call /add_target '[2, 1, 0]'

sleep 2

# set the executor on
rosservice call /set_executor_state 1

