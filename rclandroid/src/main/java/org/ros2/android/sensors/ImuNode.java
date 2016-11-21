/* Copyright 2017 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Heavily based on
// https://github.com/ros-android/android_sensors_driver/blob/groovy-devel/src/org/ros/android/android_sensors_driver/ImuPublisher.java
// Copyright (c) 2011, Chad Rockey
package org.ros2.android.sensors;

import java.util.Arrays;
import java.util.Collection;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.node.ComposableNode;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.publisher.Publisher;

public class ImuNode extends BaseComposableNode implements SensorEventListener {
  private final SensorManager sensorManager;
  private final String topic;

  private final Sensor accelerometer;
  private Publisher<sensor_msgs.msg.Imu> publisher;
  private Node node;
  private sensor_msgs.msg.Imu imu;

  public ImuNode(final String name) { this(null, name, "android/imu"); }

  public ImuNode(final SensorManager sensorManager, final String name) {
    this(sensorManager, name, "android/imu");
  }

  public ImuNode(final SensorManager sensorManager, final String name,
                 final String topic) {
    super(name);
    if (sensorManager == null) {
      sensorManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
    }

    this.sensorManager = sensorManager;

    this.topic = topic;
    this.accelerometer =
        this.sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    this.node = RCLJava.createNode(name);
    this.publisher = this.node.<sensor_msgs.msg.Imu>createPublisher(
        sensor_msgs.msg.Imu.class, this.topic);
    this.imu = new sensor_msgs.msg.Imu();
  }

  public void start() {
    this.sensorManager.registerListener(this, this.accelerometer,
                                        SensorManager.SENSOR_DELAY_NORMAL);
  }

  public void stop() { this.sensorManager.unregisterListener(this); }

  public void onAccuracyChanged(Sensor sensor, int accuracy) {}

  public void onSensorChanged(SensorEvent event) {
    if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
      this.imu.getLinearAcceleration().setX(event.values[0]);
      this.imu.getLinearAcceleration().setY(event.values[1]);
      this.imu.getLinearAcceleration().setZ(event.values[2]);

      double[] tmpCov = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
      this.imu.setLinearAccelerationCovariance(tmpCov);
      this.accelTime = event.timestamp;
    } else if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
      this.imu.getAngularVelocity().setX(event.values[0]);
      this.imu.getAngularVelocity().setY(event.values[1]);
      this.imu.getAngularVelocity().setZ(event.values[2]);
      double[] tmpCov = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
      this.imu.setAngularVelocityCovariance(tmpCov);
      this.gyroTime = event.timestamp;
    } else if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
      float[] quaternion = new float[4];
      SensorManager.getQuaternionFromVector(quaternion, event.values);
      this.imu.getOrientation().setW(quaternion[0]);
      this.imu.getOrientation().setX(quaternion[1]);
      this.imu.getOrientation().setY(quaternion[2]);
      this.imu.getOrientation().setZ(quaternion[3]);
      double[] tmpCov = {0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001};
      this.imu.setOrientationCovariance(tmpCov);
      this.quatTime = event.timestamp;
    }

    if ((this.accelTime != 0 || !this.hasAccel) &&
        (this.gyroTime != 0 || !this.hasGyro) &&
        (this.quatTime != 0 || !this.hasQuat)) {
      long time_delta_millis =
          System.currentTimeMillis() - SystemClock.uptimeMillis();
      this.imu.getHeader().setStamp(
          Time.fromMillis(time_delta_millis + event.timestamp / 1000000));
      this.imu.getHeader().setFrameId("/imu"); // TODO Make parameter

      this.publisher.publish(this.imu);

      this.imu = sensors_msgs.msg.Imu();

      this.accelTime = 0;
      this.gyroTime = 0;
      this.quatTime = 0;
    }
  }
}