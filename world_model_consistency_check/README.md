# World Model Consistency Check

A package to compare two octrees.

## Accumulated Point Cloud 2 File

This node is able to create an accumulated point cloud. 

![LAR_360_pointCloud](../docs/LAR_360_pointCloud.png)

To launch this node implement in your launch file:

```xml
<node if="$(arg record)" pkg="world_model_consistency_check" type="accumulatepointcloud2file" name="accumulatedpointcloud2file" output="screen">
    <remap from="input" to="camera/depth_registered/points"/>
    <param name="~filename" value="/home/*your_user*/auto_save.pcd" type="str"/>
    <remap from="map" to="base_link"/>
</node>
 ```

## Bounding Box

## Compare Octrees

## File 2 OctoMap Msg

## Find Unknown Space

## OctoMap Msg 2 File

## PCD 2 Point Cloud

## Point Cloud 2 File

## Test Unknown Space
