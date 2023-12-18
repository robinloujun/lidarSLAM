#!/bin/bash

rosnode kill /my_bag

sleep 5s

rosnode list | grep -v rosout | xargs rosnode kill