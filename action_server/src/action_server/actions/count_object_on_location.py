#! /usr/bin/env python
import rospy
import smach
import robot_smach_states.util.designators as ds
from robot_smach_states.util.startup import startup

class CountObjectOnLocation(smach.StateMachine):
    def __init__(self, robot, location, num_objects_designator, segmentation_area='on_top_of', object_type='', threshold=0.0):
        """ Constructor

        :param robot: robot object
        :param location: Where to look for objects?
        :param num_objects_designator: a VariableDesignator(resolve_type=int).writeable() that will store the number of objects
        :param segmentation_area: string defining where the objects are w.r.t. the entity, default = on_top_of
        :param threshold: float for classification score. Entities whose classification score is lower are ignored
            (i.e. are not added to the segmented_entity_ids_designator)
        """
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.robot = robot
        self.location = location
        self.segmentation_area = segmentation_area
        self.threshold = threshold
        self.object = object_type

        ds.checks.is_writeable(num_objects_designator)
        ds.checks.resolve_type(int)
        self.num_objects = num_objects_designator

    def count_object_on_location(self):
        res = self.robot.ed.update_kinect("{} {}".format(self.segmentation_area, self.location))
        segmented_object_ids = res.new_ids + res.updated_ids

        rospy.loginfo("Segmented %d objects" % len(segmented_object_ids))
        if segmented_object_ids:
            object_classifications = self.robot.ed.classify(ids=segmented_object_ids)

            if object_classifications:
                for idx, obj in enumerate(object_classifications):
                    rospy.loginfo("   - Object {} is a '{}' (ID: {})".format(idx, obj.type, obj.id))

                if self.threshold:
                    over_threshold = [obj for obj in object_classifications if
                    obj.probability >= self.threshold]

                dropped = {obj.id: obj.probability for obj in object_classifications if
                           obj.probability < self.threshold}
                rospy.loginfo("Dropping {l} entities due to low class. score (< {th}): {dropped}"
                              .format(th=self.threshold, dropped=dropped, l=len(dropped)))

                object_classifications = over_threshold

                list_objects = [obj for obj in object_classifications if obj['obj.type'] == self.object_type]
                num_objects = len(list_objects)
                self.num_objects_designator.write(num_objects)
            return 'done'
        else:
            return 'failed'


# Standalone testing -----------------------------------------------------------------

class TestCountObjects(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        count = ds.VariableDesignator(0)

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'COUNT',
                                                'abort': 'Aborted'})

            smach.StateMachine.add("COUNT",
                                   CountObjectOnLocation(robot, 'counter', object_type='coke', 
                                                         num_objects_designator=count.writeable()),
                                   transitions={'done': 'Done',
                                                'failed':'Aborted'})


if __name__ == "__main__":
    # rospy.init_node('gpsr_function_exec')

    startup(TestCountObjects, challenge_name="gps_function")
