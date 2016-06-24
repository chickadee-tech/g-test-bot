#!/bin/bash

protoc --python_out=. -I ../../shared/proto ../../shared/proto/polystack.proto
