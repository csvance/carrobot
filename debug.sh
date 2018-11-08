#!/bin/sh

rosservice call /roboclaw_node/set_logger_level "{logger: 'rosout', level: 'debug'}"
