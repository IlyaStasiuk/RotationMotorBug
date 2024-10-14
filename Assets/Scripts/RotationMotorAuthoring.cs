using Unity.Entities;
using Unity.Physics;
using UnityEngine;
using Unity.Mathematics;

public class RotationMotorAuthoring : MonoBehaviour
{
    [SerializeField] Transform connectedBody;

    public class Baker : Baker<RotationMotorAuthoring>
    {
        public override void Bake(RotationMotorAuthoring authoring)
        {
            DependsOn(authoring.connectedBody);

            if (!authoring.connectedBody) return;

            Constraint ballAndSocket = Constraint.BallAndSocket(1f, 1f);
            Constraint motorTwist = Constraint.MotorTwist(0f, 0.1f, 1f, 1f);

            PhysicsJoint joint = CreateJoint(authoring);
            joint.SetConstraints(new() { motorTwist, ballAndSocket });

            Entity jointEntity = CreateJointEntity(authoring);
            AddComponent(jointEntity, joint);
        }

        public Entity CreateJointEntity(RotationMotorAuthoring authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.Dynamic);
            Entity connectedEntity = GetEntity(authoring.connectedBody.gameObject, TransformUsageFlags.Dynamic);
            Entity jointEntity = CreateAdditionalEntity(TransformUsageFlags.Dynamic, false, "Joint");

            PhysicsConstrainedBodyPair bodyPair = new(entity, connectedEntity, false);
            AddComponent(jointEntity, bodyPair);

            PhysicsWorldIndex physicsWorldIndex = default;
            AddSharedComponent(jointEntity, physicsWorldIndex);

            return jointEntity;
        }

        public PhysicsJoint CreateJoint(RotationMotorAuthoring authoring)
        {
            float3 position = float3.zero;
            float3 axis = new(1, 0, 0);
            float3 secondaryAxis = new(0, 1, 0);

            RigidTransform worldFromA = Unity.Physics.Math.DecomposeRigidBodyTransform(authoring.transform.localToWorldMatrix);
            RigidTransform worldFromB = Unity.Physics.Math.DecomposeRigidBodyTransform(authoring.connectedBody.localToWorldMatrix);

            RigidTransform bFromA = math.mul(math.inverse(worldFromB), worldFromA);
            float3 connectedPosition = math.transform(bFromA, position);
            float3 connectedAxis = math.normalize(math.mul(bFromA.rot, axis));
            float3 connectedSecondaryAxis1 = math.normalize(math.mul(bFromA.rot, secondaryAxis));
            float3 connectedThirdAxis = math.normalize(math.cross(connectedAxis, connectedSecondaryAxis1));
            float3 connectedSecondaryAxis = math.normalize(math.cross(connectedThirdAxis, connectedAxis));

            BodyFrame bodyAFromJoint = new()
            {
                Position = position,
                Axis = axis,
                PerpendicularAxis = secondaryAxis,
            };

            BodyFrame bodyBFromJoint = new()
            {
                Position = connectedPosition,
                Axis = connectedAxis,
                PerpendicularAxis = connectedSecondaryAxis,
            };


            PhysicsJoint physicsJoint = default;
            physicsJoint.BodyAFromJoint = bodyAFromJoint;
            physicsJoint.BodyBFromJoint = bodyBFromJoint;
            physicsJoint.JointType = JointType.Custom;

            return physicsJoint;
        }
    }
}