function [config] = create_robot_config(q,joint_names)
    config = struct('JointName',reshape(joint_names,12,1),'JointPosition',reshape(q,12,1));
end

