#!/usr/bin/env python3
"""
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


import rospy
import object_loader_msgs.srv
import object_loader_msgs.msg
class ObjectGroupsLoader:
    def __init__(self):

        self.object_groups=rospy.get_param("~")
        rospy.logdebug("object groups:")
        for og in self.object_groups:
            rospy.logdebug("-", og)
        self.add_obj = rospy.ServiceProxy('/add_object_to_scene', object_loader_msgs.srv.AddObjects)
        self.remove_obj = rospy.ServiceProxy('/remove_object_from_scene', object_loader_msgs.srv.RemoveObjects)

        rospy.logdebug("[%s] waiting for service", rospy.get_name())
        self.add_obj.wait_for_service()
        self.remove_obj.wait_for_service()

        rospy.logdebug("[%s] connected to services", rospy.get_name())

        self.ag = rospy.Service('/add_objects_group', object_loader_msgs.srv.AddObjectsGroup, self.addGroup)
        self.rg = rospy.Service('/remove_objects_group', object_loader_msgs.srv.RemoveObjectsGroup, self.removeGroup)

        rospy.loginfo("[%s] job execution is ready", rospy.get_name())

        self.og={}
    def addGroup(self,req):
        rospy.logdebug("[%s] add group %s", rospy.get_name(),req.objects_group)
        res=object_loader_msgs.srv.AddObjectsGroupResponse()

        if not(req.objects_group in self.object_groups):
            rospy.logerr("[%s] group %s is not managed", rospy.get_name(),req.objects_group)
            res.success=False
            return res

        if (req.objects_group in self.og):
            rospy.logerr("[%s] group %s is alreay added", rospy.get_name(),req.objects_group)
            res.success=False
            return res
        objects=self.object_groups[req.objects_group]

        obj_req=object_loader_msgs.srv.AddObjectsRequest()

        for obj_param in objects:
            obj=object_loader_msgs.msg.Object()
            obj.object_type=obj_param["type"]
            obj.pose.pose.position.x=obj_param["position"][0]
            obj.pose.pose.position.y=obj_param["position"][1]
            obj.pose.pose.position.z=obj_param["position"][2]
            obj.pose.pose.orientation.x=obj_param["quaternion"][0]
            obj.pose.pose.orientation.y=obj_param["quaternion"][1]
            obj.pose.pose.orientation.z=obj_param["quaternion"][2]
            obj.pose.pose.orientation.w=obj_param["quaternion"][3]
            obj.pose.header.frame_id=obj_param["frame"]
            obj_req.objects.append(obj)

        obj_res=self.add_obj(obj_req)
        res.success=obj_res.success
        res.ids=obj_res.ids
        self.og[req.objects_group]=res.ids
        return res

    def removeGroup(self,req):
        rospy.logdebug("[%s] remove group %s", rospy.get_name(),req.objects_group)
        res=object_loader_msgs.srv.RemoveObjectsGroupResponse()

        if not(req.objects_group in self.object_groups):
            rospy.logerr("[%s] group %s is not managed", rospy.get_name(),req.objects_group)
            res.success=False
            return res

        if not(req.objects_group in self.og):
            rospy.logerr("[%s] group %s is not added", rospy.get_name(),req.objects_group)
            res.success=False
            return res

        obj_req=object_loader_msgs.srv.RemoveObjectsRequest()
        obj_req.obj_ids=self.og[req.objects_group]
        obj_res=self.remove_obj(obj_req)
        res.success=obj_res.success
        self.og.pop(req.objects_group)
        return res

def main():
    rospy.init_node('object_groups_loader')



    server =ObjectGroupsLoader()
    rospy.spin()


if __name__ == '__main__':
    main()
