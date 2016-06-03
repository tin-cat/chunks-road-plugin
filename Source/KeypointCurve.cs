using System.Collections.Generic;
using Chunks.Entities;
using Chunks.Geometry;

namespace Road
{
    public abstract class KeypointCurve : Component
    {
        protected const float Epsilon = 0.0001f;

        protected struct Keypoint
        {
            public Vector Position;
            public Vector Tangent;
            public Vector Normal;
        }

        protected Keypoint Start => _start;
        protected Keypoint End => _end;

        private Keypoint _start;
        private Keypoint _end;
        private float _length;
        private float _invLength;
        private bool _invalidated;

        /// <summary>
        /// Next KeypointCurve to connect to.
        /// </summary>
        public KeypointCurve Next { get; set; }

        /// <summary>
        /// Total length of the curve.
        /// </summary>
        public float Length
        {
            get
            {
                if (_invalidated) UpdateCurve();
                return _length;
            }
        }

        /// <summary>
        /// One divided by Length, for optimizations.
        /// </summary>
        public float InverseLength
        {
            get
            {
                if (_invalidated) UpdateCurve();
                return _invLength;
            }
        }

        public void Invalidate()
        {
            _invalidated = true;
        }

        public IEnumerable<float> GetDeltas(float deltaAngleRadians, float minDist, float maxDist)
        {
            if (_invalidated) UpdateCurve();
            return OnGetDeltas(deltaAngleRadians, minDist, maxDist);
        }

        protected abstract IEnumerable<float> OnGetDeltas(float deltaAngleRadians, float minDist, float maxDist);
        
        private bool UpdateKeypoint(ref Keypoint keypoint)
        {
            var changed = false;

            var pos = Transform.Position;
            var tan = Transform.Forward;
            var nrm = Transform.Up;

            if (!pos.Equals(keypoint.Position, Epsilon))
            {
                changed = true;
                keypoint.Position = pos;
            }

            if (!tan.Equals(keypoint.Tangent, Epsilon))
            {
                changed = true;
                keypoint.Tangent = tan;
            }

            if (!nrm.Equals(keypoint.Normal, Epsilon))
            {
                changed = true;
                keypoint.Normal = nrm;
            }

            return changed;
        }

        private void UpdateCurve()
        {
            _invalidated = false;

            var startChanged = UpdateKeypoint(ref _start);
            var endChanged = Next == null && !UpdateKeypoint(ref _end) || Next != null && !Next.UpdateKeypoint(ref _end);

            OnUpdateCurve(out _length);
            _invLength = 1f / _length;
        }

        protected abstract void OnUpdateCurve(out float length);

        public Vector GetPosition(float t)
        {
            if (_invalidated) UpdateCurve();
            return OnGetPosition(t);
        }

        protected abstract Vector OnGetPosition(float t);

        public Quaternion GetRotation(float t)
        {
            if (_invalidated) UpdateCurve();
            return OnGetRotation(t);
        }

        protected abstract Quaternion OnGetRotation(float t);

        public bool IsWithinRange(Vector pos, float maxDist, out float t, out float dist)
        {
            if (_invalidated) UpdateCurve();
            return OnIsWithinRange(pos, maxDist, out t, out dist);
        }

        protected abstract bool OnIsWithinRange(Vector pos, float maxDist, out float t, out float dist);

        public bool IsApproxWithinLineRange(Vector start, Vector end, float maxDist, out float tLine, out float tCurve, out float dist)
        {
            if (_invalidated) UpdateCurve();
            return OnIsApproxWithinLineRange(start, end, maxDist, out tLine, out tCurve, out dist);
        }

        protected abstract bool OnIsApproxWithinLineRange(Vector start, Vector end, float maxDist, out float tLine, out float tCurve, out float dist);

        public bool IsApproxOverlapping(KeypointCurve other, float range)
        {
            if (_invalidated) UpdateCurve();
            return OnIsApproxOverlapping(other, range);
        }

        protected abstract bool OnIsApproxOverlapping(KeypointCurve other, float range);
    }
}
