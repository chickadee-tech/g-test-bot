#!/bin/bash

protoc --python_out=. -I ../../shared/proto ../../shared/proto/flightstack.proto
