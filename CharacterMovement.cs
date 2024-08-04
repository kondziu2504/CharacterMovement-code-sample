using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using PixelFiberGames.Utility;
using UnityEngine;
using Utility;

namespace PixelFiberGames.MovementSystems
{
    public class CharacterMovement : MonoBehaviour
    {
        public Vector3 Velocity => velocity;
        public Vector3 Position => rigidbody.position;

        [SerializeField] private new Rigidbody rigidbody;
        [SerializeField] private CapsuleCollider capsuleCollider;
        [SerializeField] private LayerMask groundLayer;
        [SerializeField] private float drag = 1f;
        [SerializeField] private float gravity = 10f;
        [SerializeField] private float stepHeight = 0.5f;        

        private Vector3 motionInput;        
        private Vector3 velocity;        
        private List<Collider> ignoredColliders = new();

        private SlopeMovementCorrection slopeMovementCorrection;
        private OverlapResolver overlapResolver;
        private MovementAlongSurface movementAlongSurface;
        private GravityFall gravityFall;
        private StepCorrection stepCorrection;

        private void Awake()
        {
            Debug.Assert(!groundLayer.ContainsLayer(rigidbody.gameObject.layer));
            rigidbody.isKinematic = true;
            
            CreateLogic();
        }

        private void OnValidate()
        {
            CreateLogic();
        }

        private void FixedUpdate()
        {
            var newPosition = CalculateMovementDestination(rigidbody.position, motionInput);
            rigidbody.MovePosition(newPosition);
            motionInput = Vector3.zero;
        }

        private void CreateLogic()
        {
            slopeMovementCorrection = new SlopeMovementCorrection(new GroundCheck(capsuleCollider, 0.05f, groundLayer));
            overlapResolver = new OverlapResolver(capsuleCollider, groundLayer, 0.015f);
            movementAlongSurface = new MovementAlongSurface(capsuleCollider, groundLayer, 3, 0.01f);
            gravityFall = new GravityFall(new GroundCheck(capsuleCollider, 0.02f, groundLayer), gravity);
            stepCorrection = new StepCorrection(capsuleCollider, stepHeight, 0.05f, 0.025f, groundLayer);
        }

        public void AddForce(Vector3 force) => velocity += force;
        public void SetDrag(float drag) => this.drag = drag;
        public void IgnoreColliders(params Collider[] colliders) => ignoredColliders.AddRange(colliders);
        public void UnignoreColliders(params Collider[] colliders) => ignoredColliders.RemoveAll(x => colliders.Contains(x));

        public void Stop()
        {
            motionInput = Vector3.zero;
            velocity = Vector3.zero;
        }

        public void Move(Vector3 motion)
        {
            motionInput += motion;
        }

        public void MoveToPosition(Vector3 position)
        {
            motionInput = (position - rigidbody.position).OnlyXZ();
        }


        public bool IsClearPath(Vector3 direction, float maxDistance, out float coveredDistance)
        {
            var motionResult = SimulateStraightMotion(direction, maxDistance);
            coveredDistance = (motionResult.Destination - rigidbody.position).magnitude;
            bool pathClear = motionResult.Obstacle == null;
            return pathClear;
        }


        private Vector3 CalculateMovementDestination(Vector3 startPosition, Vector3 motion)
        {
            Vector3 newPosition;
            
            using (CreateIgnoreCollidersContext())
            {
                var totalMotion = slopeMovementCorrection.CorrectMotion(startPosition, motion) + velocity * Time.fixedDeltaTime;

                newPosition = startPosition;
                newPosition = overlapResolver.Resolve(newPosition);
                newPosition = stepCorrection.CorrectPosition(newPosition, totalMotion, out var stepCorrected);
                newPosition = movementAlongSurface.CalculatePosition(newPosition, totalMotion);

                velocity *= (1f - Time.fixedDeltaTime * drag);
                if (stepCorrected)
                    velocity.y = 0f;
                else
                    velocity = gravityFall.AdjustVelocity(newPosition, velocity);
            }

            return newPosition;
        }

        public Vector3 CalculateDashDestination(Vector3 direction, float maxDistance, Func<RaycastHit, bool> isHitValid = null) 
            => SimulateStraightMotion(direction, maxDistance, isHitValid).Destination;

        public StraightMotionResult SimulateStraightMotion(Vector3 direction, float maxDistance, Func<RaycastHit, bool> isHitValid = null)
        {
            direction = (direction.OnlyXZ()).normalized;
                        
            Vector3 position = rigidbody.position;
            Transform obstacle = null;
            Vector3 obstacleContactPoint = Vector3.zero;
            bool stepCorrected = false;

            using (CreateIgnoreCollidersContext())
            {                
                position = overlapResolver.Resolve(position);
                const float DistancePerStep = 0.5f;
                while (maxDistance > 0.01f)
                {
                    var distanceToCover = Mathf.Min(maxDistance, DistancePerStep);
                    maxDistance -= distanceToCover;

                    var motion = direction * distanceToCover;
                    var totalMotion = slopeMovementCorrection.CorrectMotion(position, motion);
                    position = stepCorrection.CorrectPosition(position, totalMotion, out stepCorrected);

                    var hits = PhysicsUtils.CapsuleDimensionsCastAll(capsuleCollider.GetDimensionsAt(position), totalMotion.normalized, distanceToCover, groundLayer, QueryTriggerInteraction.Ignore);

                    if (isHitValid != null)
                        hits = hits.Where(x => isHitValid(x)).ToArray();

                    Array.Sort(hits, (RaycastHit x, RaycastHit y) => x.distance.CompareTo(y.distance));
                    
                    if (hits.Length > 0)
                    {
                        var hit = hits[0];
                        position += direction * Mathf.Max(0f, hit.distance - 0.01f);                        
                        obstacleContactPoint = hit.point;
                        break;
                    }
                    else
                    {                        
                        position += direction * distanceToCover;
                    }
                }
            }

            return new StraightMotionResult(position, obstacle, obstacleContactPoint, stepCorrected);
        }

        public class StraightMotionResult
        {
            public Vector3 Destination { get; }
            public Transform Obstacle { get; }
            public Vector3 ObstacleContactPoint { get; }
            public bool StepCorrected { get; }

            public StraightMotionResult(Vector3 destination, Transform obstacle, Vector3 obstacleContactPoint, bool stepCorrected)
            {
                Destination = destination;
                Obstacle = obstacle;
                ObstacleContactPoint = obstacleContactPoint;
                StepCorrected = stepCorrected;
            }
        }


        private IgnoreCollidersContext CreateIgnoreCollidersContext() => new IgnoreCollidersContext(ignoredColliders);
       
        private class IgnoreCollidersContext : IDisposable
        {
            private readonly List<Collider> ignoredColliders;
            private bool disposed = false;
            private bool[] savedStates;

            public IgnoreCollidersContext(List<Collider> ignoredColliders)
            {
                this.ignoredColliders = ignoredColliders;

                savedStates = ignoredColliders.Select(x => x.enabled).ToArray();
                foreach (var ignoredCollider in ignoredColliders)
                    ignoredCollider.enabled = false;
            }

            public void Dispose()
            {
                if (disposed)
                    return;
                disposed = true;

                for (int i = 0; i < savedStates.Length; i++)
                    ignoredColliders[i].enabled = savedStates[i];
            }
        }
    }
}
