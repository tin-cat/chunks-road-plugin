using System;
using System.Linq;
using Chunks;
using Chunks.Audio;
using Chunks.Entities;
using Chunks.Geometry;
using Chunks.Graphics;
using Chunks.Interface;
using Chunks.Plugins;
using Explosives;

namespace Road
{
    public class Car : Component
    {
        public const float DefaultFriction = 0.025f;
        public const float DefaultMinTangentialVelocity = 0.1f;

        private readonly SoundEffect _chainSound = Plugin.GetResource<SoundEffect>("Road.ChainLift");
        private readonly SoundEffect _rollingSound = Plugin.GetResource<SoundEffect>("Road.Rolling");

        private Holder _holder;

        private ModelRenderer _meshRenderer;
        private TrackSegment _curTrack;
        private SoundEmitter _soundEmitter;
        private SoundEmitter _rollingEmitter;

        public float TrackSegmentProgress { get; private set; }

        public float Length => 1f/World.ChunkSize;

        public TrackSegment CurrentTrackSegment
        {
            get { return _curTrack; }
            private set
            {
                _curTrack?.RemoveCar(this);
                _curTrack = value;
                _curTrack?.AddCar(this);
            }
        }
        
        public Car NextCar { get; private set; }
        public Car PrevCar { get; private set; }
        public bool AttachedToNextCar { get; private set; }

        public float TopSpeed { get; set; }
        public float Torque { get; set; }
        public float Friction { get; set; }
        public float MinTangentialVelocity { get; set; }

        public float TangentialVelocity { get; private set; }

        private event Action<float> PhysicsUpdated;

        public Car()
        {
            TopSpeed = 2f;
            Friction = DefaultFriction;
            MinTangentialVelocity = DefaultMinTangentialVelocity;
        }

        protected override void OnInitialize()
        {
            Name = "Car";

            _meshRenderer = CreateChildWithComponent<ModelRenderer>();
            _meshRenderer.Model = Plugin.GetResource<Model>("Road.Car");
            _meshRenderer.Material = BlockMaterial.Default;

            _meshRenderer.Transform.LocalScale = Vector.One / World.ChunkSize;

            _holder = AddComponent<Holder>();
            _holder.HoldScale = Vector.One*.5f;
            _holder.HoldOffset = (Vector.UnitY/8.0625f)/World.ChunkSize;

            _soundEmitter = AddComponent<SoundEmitter>();
            _rollingEmitter = CreateChildWithComponent<SoundEmitter>();
        }

        public void AttachToTrack(TrackSegment track, float progress)
        {
            if (CurrentTrackSegment != null)
            {
                throw new NotImplementedException();
            }

            TrackSegmentProgress = progress;
            TangentialVelocity = 0f;

            CurrentTrackSegment = track;

            UpdatePosition();
            UpdateAdjacentCars();
        }

        private void UpdateAdjacentCars()
        {
            var oldNextCar = NextCar;
            var oldPrevCar = PrevCar;

            if (CurrentTrackSegment == null)
            {
                NextCar = null;
                PrevCar = null;
            }
            else
            {
                NextCar = CurrentTrackSegment.FindNextCar(TrackSegmentProgress);
                PrevCar = CurrentTrackSegment.FindPrevCar(TrackSegmentProgress);
            }

            if (NextCar != oldNextCar && NextCar != null)
            {
                if (oldNextCar != null && AttachedToNextCar)
                {
                    NextCar.PhysicsUpdated -= OnHeadCarPhysicsUpdated;
                }

                NextCar.PrevCar = this;
            }

            if (PrevCar != oldPrevCar && PrevCar != null)
            {
                if (PrevCar.NextCar != null && PrevCar.AttachedToNextCar)
                {
                    PrevCar.NextCar.PhysicsUpdated -= PrevCar.OnHeadCarPhysicsUpdated;
                }

                PrevCar.NextCar = this;
            }

            AttachedToNextCar = AttachedToNextCar && oldNextCar == NextCar;
        }

        public bool AttachBehindCar(Car next)
        {
            AttachToTrack(next.CurrentTrackSegment, next.TrackSegmentProgress - 1f / World.ChunkSize);

            if (NextCar != next)
            {
                Debug.Log("Failed to attach cars");
                return false;
            }

            AttachedToNextCar = true;

            NextCar.PhysicsUpdated += OnHeadCarPhysicsUpdated;

            return true;
        }

        public void DetachFromTrack()
        {
            if (CurrentTrackSegment == null) return;

            UpdatePositionFinal();
            CurrentTrackSegment = null;

            if (NextCar != null) NextCar.PrevCar = PrevCar;
            if (PrevCar != null) PrevCar.NextCar = NextCar;

            NextCar = null;
            PrevCar = null;

            _soundEmitter.Stop();
            _rollingEmitter.Stop();

            var explosive = AddComponent<Explosive>();

            explosive.Exploded += expl =>
            {
                StopRiding();
                _holder.Release();
                _meshRenderer.Enabled = false;
            };

            explosive.Power *= 0.5f;
            explosive.CollisionForceThreshold = 1f / 64f;
            explosive.Holdable.Physics.Velocity = Transform.Forward * TangentialVelocity * MathF.Random(0.9f, 1f);
            explosive.LightFuse();

            GetComponent<AABBPhysics>().Gravity *= 0.333f;
        }

        public float GetDistanceSquared(Vector pos)
        {
            var localPos = _meshRenderer.Transform.InverseTransformPosition(pos);

            var min = new Vector(-0.5f, 0f, -0.5f);
            var max = new Vector(0.5f, 0.5f, 0.5f);

            var xDist = localPos.X < min.X ? min.X - localPos.X : localPos.X > max.X ? localPos.X - max.X : 0f;
            var yDist = localPos.Y < min.Y ? min.Y - localPos.Y : localPos.Y > max.Y ? localPos.Y - max.Y : 0f;
            var zDist = localPos.Z < min.Z ? min.Z - localPos.Z : localPos.Z > max.Z ? localPos.Z - max.Z : 0f;

            return _meshRenderer.Transform.TransformVector(new Vector(xDist, yDist, zDist)).LengthSquared;
        }

        private void UpdatePositionFinal()
        {
            if (CurrentTrackSegment == null) return;

            Vector pos; Quaternion rotation;
            CurrentTrackSegment.GetTrackTransformAtDelta(TrackSegmentProgress, out pos, out rotation);

            Transform.Position = pos;
            Transform.Rotation = rotation;
        }

        private void UpdateCurrentTrackSegment()
        {
            var changedSeg = false;

            while (TrackSegmentProgress >= CurrentTrackSegment.Length || TrackSegmentProgress > 0f && CurrentTrackSegment.Next == null)
            {
                if (CurrentTrackSegment.Next == null)
                {
                    DetachFromTrack();
                    return;
                }

                TrackSegmentProgress -= CurrentTrackSegment.Length;
                CurrentTrackSegment = CurrentTrackSegment.Next;
                changedSeg = true;
            }

            while (TrackSegmentProgress < 0f)
            {
                if (CurrentTrackSegment.Prev == null)
                {
                    DetachFromTrack();
                    return;
                }

                CurrentTrackSegment = CurrentTrackSegment.Prev;
                TrackSegmentProgress += CurrentTrackSegment.Length;
                changedSeg = true;
            }

            if (changedSeg) UpdateAdjacentCars();
        }
        
        private void UpdatePosition()
        {
            UpdateCurrentTrackSegment();
            UpdatePositionFinal();
        }

        private void OnHeadCarPhysicsUpdated(float velocity)
        {
            Move(velocity*World.PhysicsDeltaTime);

            PhysicsUpdated?.Invoke(velocity);
        }

        private void Move(float delta)
        {
            if (CurrentTrackSegment == null) return;
            
            TrackSegmentProgress += delta;
            UpdatePosition();
        }

        public void StartRiding()
        {
            var camera = World.CameraRig;

            var forward = camera.HeadTransform.Forward;
            forward = new Vector(forward.X, 0f, forward.Z).NormalizedSafe;

            var offset = camera.HeadTransform.LocalPosition*new Vector(1f, 0f, 1f);

            var scale = 1f/World.ChunkSize;
            var rotation = Quaternion.LookRotation(forward, Vector.UnitY).Inverse;

            Transform.Add(camera.Transform, false);
            camera.ResetTransform(1f / World.ChunkSize, rotation * -offset * scale);

            camera.Transform.LocalRotation = rotation;
        }

        public void StopRiding()
        {
            var camera = Transform.Children
                .Select(x => x.Entity.GetComponent<CameraRig>())
                .FirstOrDefault(x => x != null);

            if (camera == null) return;

            camera.Transform.DetachFromParent(false);
            camera.ResetTransform();
        }

        protected override void OnDestroy()
        {
            StopRiding();
        }

        public int GetAttachedCarCount()
        {
            var count = 1;
            var next = this;
            while (next != null && next.AttachedToNextCar)
            {
                ++count;
                next = next.NextCar;
                if (next == this) throw new Exception("Car train has no head!");
                if (count > 512) throw new Exception("Infinite loop (A)");
            }

            next = PrevCar;
            while (next != null && next.AttachedToNextCar)
            {
                ++count;
                next = next.PrevCar;
                if (count > 512) throw new Exception("Infinite loop (B)");
            }

            return count;
        }

        public Car GetTrainHead()
        {
            var count = 0;
            var next = this;
            while (next.AttachedToNextCar && next.NextCar != null)
            {
                next = next.NextCar;
                if (next == this) throw new Exception("Car train has no head!");
                if (++count > 512) throw new Exception("Infinite loop (C)");
            }

            return next;
        }

        protected override void OnPhysicsUpdate()
        {
            if (CurrentTrackSegment == null) return;

            var range = 4f/World.ChunkSize;

            var canInteractWithNext = NextCar != null
                && (NextCar.CurrentTrackSegment == CurrentTrackSegment && NextCar.TrackSegmentProgress > TrackSegmentProgress
                || NextCar.CurrentTrackSegment == CurrentTrackSegment.Next)
                && NextCar.Transform.Forward.Dot(Transform.Forward) > 0f
                && (NextCar.Transform.Position - Transform.Position).LengthSquared <= range * range;

            if (canInteractWithNext)
            {
                var nextBack = NextCar.Transform.Position - NextCar.Transform.Forward*NextCar.Length*.5f;
                var thisFront = Transform.Position + Transform.Forward*Length*.5f;

                var dist = Transform.Forward.Dot(nextBack - thisFront) - 1f / 256f;

                if (dist < 0f && dist > -Length * .5f || AttachedToNextCar)
                {
                    NextCar.Move(dist * -.25f);
                    Move(dist*0.25f);

                    if (!AttachedToNextCar)
                    {
                        var thisCount = GetAttachedCarCount();
                        var thatCount = NextCar.GetAttachedCarCount();

                        var nextHead = NextCar.GetTrainHead();
                        var force = TangentialVelocity*thisCount -
                            nextHead.TangentialVelocity*thatCount;

                        if (!float.IsNaN(force) && !float.IsInfinity(force))
                        {
                            TangentialVelocity -= force / thisCount * 0.75f;
                            nextHead.TangentialVelocity += force / thatCount * 0.75f;
                        }
                    }
                }

                if (AttachedToNextCar && NextCar != null)
                {
                    TangentialVelocity = NextCar.TangentialVelocity;
                    _soundEmitter.Enabled = false;
                    _rollingEmitter.Enabled = false;
                    return;
                }

                if (CurrentTrackSegment == null) return;
            }

            var anyOnHill = false;
            var anyOnBrake = false;
            var anyOnBooster = false;
            var car = this;
            var count = 0;
            var accel = 0f;

            _soundEmitter.Enabled = true;
            _rollingEmitter.Enabled = true;

            do
            {
                var segment = car.CurrentTrackSegment;
                var grad = segment.GetTrackGradientAtDelta(car.TrackSegmentProgress);

                ++count;
                accel -= grad * 9.81f / 16f * World.PhysicsDeltaTime;

                anyOnHill |= segment.Type == TrackType.ChainLift && grad > float.Epsilon;
                anyOnBrake |= segment.Type == TrackType.Brake;
                anyOnBooster |= segment.Type == TrackType.Booster;

                car = car.PrevCar;
            } while (car != null && car != this && car.AttachedToNextCar);
            
            TangentialVelocity += accel/count;
            TangentialVelocity *= MathF.Pow(1f - Friction, World.PhysicsDeltaTime);

            var torque = Math.Max(Torque, anyOnBooster ? 0.5f : 0f);
            var accelScale = 1f - MathF.Pow(0.25f, World.PhysicsDeltaTime);

            TangentialVelocity += Math.Max(0f, TopSpeed - TangentialVelocity)*accelScale*torque;

            if (anyOnBrake && TangentialVelocity > .125f)
            {
                TangentialVelocity += (.125f - TangentialVelocity)*accelScale*2f;
            }
            else if (anyOnHill && TangentialVelocity < .125f)
            {
                if (!_soundEmitter.IsPlaying || _soundEmitter.SoundEffect != _chainSound)
                {
                    _soundEmitter.Play(_chainSound);
                }
                
                TangentialVelocity = .125f;
            }
            else
            {
                _soundEmitter.Stop();
            }
            
            if (!_rollingEmitter.IsPlaying || _rollingEmitter.SoundEffect != _rollingSound)
            {
                _rollingEmitter.Play(_rollingSound);
            }

            var pitch = Math.Abs(TangentialVelocity) + 1f - 0.125f;

            _rollingEmitter.Volume = MathF.Clamp01((float) Math.Pow(Math.Log(Math.Abs(TangentialVelocity) + 1d, Math.E), 0.5));

            _soundEmitter.Pitch = pitch;
            _rollingEmitter.Pitch = pitch;

            if (TangentialVelocity < MinTangentialVelocity)
                TangentialVelocity = MinTangentialVelocity;

            if (!anyOnBooster && !anyOnBrake && TangentialVelocity > 1f)
                DetachFromTrack();

            Move(TangentialVelocity * World.PhysicsDeltaTime);

            PhysicsUpdated?.Invoke(TangentialVelocity);
        }
    }
}
