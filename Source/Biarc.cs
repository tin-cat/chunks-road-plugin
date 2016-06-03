using System;
using System.Collections.Generic;
using System.Linq;
using Chunks;
using Chunks.Geometry;

namespace Road
{
    /// <summary>
    /// Describes a curve defined by two end points with tangents connected by a
    /// pair of arcs that follow the surface of two spheres.
    /// </summary>
    public class Biarc : KeypointCurve
    {
        protected struct Arc
        {
            public Vector Center;
            public float Radius;
            public Vector Axis1;
            public Vector Axis2;
            public Vector Axis3;
            public float Angle;
            public float ArcLength;
            public Quaternion StartRotation;
            public Quaternion DeltaRotation;
            public float RollDelta;
        }
        
        private Arc _arc1;
        private Arc _arc2;

        protected Arc Arc1 => _arc1;
        protected Arc Arc2 => _arc2;

        protected override IEnumerable<float> OnGetDeltas(float deltaAngleRadians, float minDist, float maxDist)
        {
            var mid = _arc1.ArcLength/(_arc1.ArcLength + _arc2.ArcLength);

            return GetDeltas(true, deltaAngleRadians, minDist, maxDist, 0f, mid)
                .Concat(GetDeltas(false, deltaAngleRadians, minDist, maxDist, mid, 1f - mid));
        }

        private IEnumerable<float> GetDeltas(bool first, float deltaAngle,
            float minDist, float maxDist, float offset, float scale)
        {
            var arc = first ? _arc1 : _arc2;

            if (scale <= 0f)
            {
                if (!first) yield return offset;
                yield break;
            }

            int count;
            float dist;

            if (arc.Radius <= 0f)
            {
                dist = maxDist;
            }
            else
            {
                var angle = arc.ArcLength/arc.Radius;
                count = MathF.FloorToInt(angle / deltaAngle + 1f);
                dist = MathF.Clamp(arc.ArcLength / count, minDist, maxDist);
            }

            count = MathF.FloorToInt(arc.ArcLength / dist + 1f);
            var delta = scale / count;

            for (var i = 0; i < count; ++i)
            {
                yield return offset + delta*i;
            }

            if (!first) yield return offset + scale;
        }

        private static void ComputeArcs(Vector point, Vector tangent, Vector pointToMid, ref Arc arc)
        {
            var norm = pointToMid.Cross(tangent);
            var perp = tangent.Cross(norm);

            var denom = 2f*perp.Dot(pointToMid);

            if (Math.Abs(denom) < Epsilon)
            {
                arc.Center = point + pointToMid * .5f;
                arc.Radius = 0f;
                arc.Angle = 0f;
                return;
            }

            var centerDist = pointToMid.LengthSquared/denom;
            arc.Center = point + perp * centerDist;

            var perpMag = perp.Length;
            arc.Radius = Math.Abs(centerDist * perpMag);

            if (arc.Radius < Epsilon)
            {
                arc.Angle = 0f;
                return;
            }

            var invRad = 1f/arc.Radius;
            var centreToMid = point - arc.Center;
            var centreToEndDir = centreToMid * invRad;
            var centerToMidDir = (centreToMid + pointToMid) * invRad;
            var twist = perp.Dot(pointToMid);

            arc.Angle = (float) Math.Acos(centreToEndDir.Dot(centerToMidDir)) * Math.Sign(twist);
        }

        private static void FindParameters(ref Keypoint start, ref Keypoint end, ref Arc arc1, ref Arc arc2)
        {
            var p1 = start.Position;
            var p2 = end.Position;

            var t1 = start.Tangent;
            var t2 = end.Tangent;

            var v = p2 - p1;
            var t = t1 + t2;
            var vMag2 = v.LengthSquared;

            var vDotT = v.Dot(t);
            var denom = 2f*(1f - t1.Dot(t2));

            float d;
            if (denom < Epsilon)
            {
                var vDotT2 = v.Dot(t2);

                if (Math.Abs(vDotT2) < Epsilon)
                {
                    var mag = MathF.Sqrt(vMag2);
                    var invMag2 = 1f/vMag2;

                    var planeNormal = v.Cross(t2);
                    var perpAxis = planeNormal.Cross(v);

                    var rad = mag*0.25f;

                    var centreToP1 = v*-0.25f;

                    arc1.Center = p1 - centreToP1;
                    arc1.Radius = rad;
                    arc1.Axis1 = centreToP1;
                    arc1.Axis2 = perpAxis * rad * invMag2;
                    arc1.Angle = MathF.Pi;
                    arc1.ArcLength = MathF.Pi*rad;

                    arc2.Center = p2 + centreToP1;
                    arc2.Radius = rad;
                    arc2.Axis1 = centreToP1 * -1f;
                    arc2.Axis2 = perpAxis * -rad * invMag2;
                    arc2.Angle = MathF.Pi;
                    arc2.ArcLength = MathF.Pi * rad;

                    return;
                }

                d = vMag2 / (4f * vDotT2);
            }
            else
            {
                var discrim = vDotT*vDotT + denom*vMag2;
                d = (-vDotT + MathF.Sqrt(discrim)) / denom;
            }

            var pm = (p1 + p2 + (t1 - t2)*d) * 0.5f;

            var p1ToPm = pm - p1;
            var p2ToPm = pm - p2;

            ComputeArcs(p1, t1, p1ToPm, ref arc1);
            ComputeArcs(p2, t2, p2ToPm, ref arc2);

            if (d < 0f)
            {
                arc1.Angle = Math.Sign(arc1.Angle) * MathF.Pi * 2f - arc1.Angle;
                arc2.Angle = Math.Sign(arc2.Angle) * MathF.Pi * 2f - arc2.Angle;
            }

            arc1.Axis1 = p1 - arc1.Center;
            arc1.Axis2 = t1 * arc1.Radius;
            arc1.ArcLength = arc1.Radius == 0f ? p1ToPm.Length : Math.Abs(arc1.Radius * arc1.Angle);

            arc2.Axis1 = p2 - arc2.Center;
            arc2.Axis2 = t2 * -arc2.Radius;
            arc2.ArcLength = arc2.Radius == 0f ? p2ToPm.Length : Math.Abs(arc2.Radius * arc2.Angle);
        }

        private static void FindRotation(ref Keypoint keyPoint, ref Arc arc)
        {
            arc.StartRotation = Quaternion.LookRotation(keyPoint.Tangent, keyPoint.Normal);

            if (arc.ArcLength < Epsilon || arc.Radius == 0f)
            {
                arc.DeltaRotation = Quaternion.Identity;
                return;
            }
            
            var angle = arc.Angle*180f/MathF.Pi;

            arc.DeltaRotation = Quaternion.AxisAngle(arc.Axis3, angle);
        }

        protected override void OnUpdateCurve(out float length)
        {
            var start = Start;
            var end = End;

            FindParameters(ref start, ref end, ref _arc1, ref _arc2);

            _arc1.Axis3 = _arc1.Axis1.Cross(_arc1.Axis2).NormalizedSafe;
            _arc2.Axis3 = _arc2.Axis1.Cross(_arc2.Axis2).NormalizedSafe;

            FindRotation(ref start, ref _arc1);
            FindRotation(ref end, ref _arc2);

            var mid1 = _arc1.DeltaRotation*_arc1.StartRotation;
            var mid2 = _arc2.DeltaRotation*_arc2.StartRotation;

            var fromX = mid1*Vector.UnitX;
            var fromY = mid1*Vector.UnitY;
            var destX = mid2*Vector.UnitX;

            var x = destX.Dot(fromX);
            var y = destX.Dot(fromY);

            var delta = MathF.Atan2(y, x) * 180f / MathF.Pi;

            length = _arc1.ArcLength + _arc2.ArcLength;
            var invLength = 1f/length;

            _arc1.RollDelta = delta * _arc1.ArcLength * invLength;
            _arc2.RollDelta = -delta * _arc2.ArcLength * invLength;
        }

        protected override Vector OnGetPosition(float t)
        {
            var length = Length;
            var delta = t*length;

            if (delta < _arc1.ArcLength)
            {
                if (_arc1.ArcLength < Epsilon) return _arc1.Center + _arc1.Axis1;

                var arcFrac = delta/_arc1.ArcLength;
                if (_arc1.Radius == 0f) return _arc1.Center + _arc1.Axis1 * (1f - arcFrac * 2f);

                var angle = _arc1.Angle*arcFrac;
                var sinRot = MathF.Sin(angle);
                var cosRot = MathF.Cos(angle);

                return _arc1.Center + _arc1.Axis1 * cosRot + _arc1.Axis2 * sinRot;
            }
            else
            {
                if (_arc2.ArcLength < Epsilon) return _arc2.Center + _arc2.Axis2;

                var arcFrac = (delta - _arc1.ArcLength) / _arc2.ArcLength;
                if (_arc2.Radius == 0f) return _arc2.Center + _arc2.Axis1 * (arcFrac * 2f - 1f);

                var angle = _arc2.Angle * (1f - arcFrac);
                var sinRot = MathF.Sin(angle);
                var cosRot = MathF.Cos(angle);

                return _arc2.Center + _arc2.Axis1 * cosRot + _arc2.Axis2 * sinRot;
            }
        }

        protected override Quaternion OnGetRotation(float t)
        {
            var length = Length;
            var delta = t*length;

            if (delta < _arc1.ArcLength)
            {
                delta = delta / _arc1.ArcLength;

                var roll = Quaternion.AxisAngle(Vector.UnitZ, _arc1.RollDelta*delta);
                return Quaternion.Slerp(Quaternion.Identity, _arc1.DeltaRotation, delta) * _arc1.StartRotation * roll;
            }
            else
            {
                delta = 1f - (delta - _arc1.ArcLength) / _arc2.ArcLength;

                var roll = Quaternion.AxisAngle(Vector.UnitZ, _arc2.RollDelta*delta);
                return Quaternion.Slerp(Quaternion.Identity, _arc2.DeltaRotation, delta) * _arc2.StartRotation * roll;
            }
        }

        private bool IsWithinRange(ref Arc arc, bool flip, float tScale, Vector pos, float maxDist, ref float t, ref float dist)
        {
            Vector diff;

            if (arc.Radius == 0f)
            {
                diff = pos - arc.Center;
                var dot = MathF.Clamp01((1f - diff.Dot(arc.Axis1)/arc.Axis1.LengthSquared)*0.5f);

                t = dot * arc.ArcLength * tScale;
            }
            else
            {
                diff = pos - arc.Center;
                var dist2 = diff.LengthSquared;
                var planeDist = diff.Dot(arc.Axis3);
                if (Math.Abs(planeDist) > maxDist) return false;

                var maxRad = arc.Radius + maxDist;
                if (dist2 > maxRad * maxRad) return false;

                var minRad = arc.Radius - maxDist;
                if (minRad > 0f && dist2 < minRad * minRad) return false;

                var planePos = (pos - planeDist * arc.Axis3 - arc.Center).Normalized * arc.Radius;

                var x = arc.Axis1.Dot(planePos);
                var y = arc.Axis2.Dot(planePos);
                var ang = MathF.Clamp(MathF.Atan2(y, x), 0f, arc.Angle);

                t = ang * arc.Radius * tScale;
            }

            if (flip) t = 1f - t;

            var nearest = GetPosition(t);

            dist = (nearest - pos).Length;

            return dist <= maxDist;
        }

        protected override bool OnIsWithinRange(Vector pos, float maxDist, out float t, out float dist)
        {
            var tScale = 1f/Length;

            float t1 = 0f, t2 = 0f, d1 = float.MaxValue, d2 = float.MaxValue;
            var b1 = IsWithinRange(ref _arc1, false, tScale, pos, maxDist, ref t1, ref d1);
            var b2 = IsWithinRange(ref _arc2, true, tScale, pos, maxDist, ref t2, ref d2);

            t = 0;
            dist = float.MaxValue;

            if (!b1 && !b2) return false;

            if (!b1 || b2 && d2 < d1)
            {
                t = t2;
                dist = d2;
                return true;
            }

            t = t1;
            dist = d1;
            return true;
        }
        
        private float ArcLineProximityTest(ref Arc arc, ref Vector start, ref Vector end, float maxDist, out float arcDelta, out float dist)
        {
            arcDelta = float.PositiveInfinity;
            dist = float.PositiveInfinity;

            if (arc.Radius == 0f) return float.PositiveInfinity;

            float lineDelta;
            if (!GeometryHelper.ApproxWithinLineRange(ref arc.Center, ref arc.Axis1, ref arc.Axis3, arc.Radius, ref start, ref end, maxDist, out lineDelta)) return float.PositiveInfinity;
            return !IsWithinRange(start + (end - start)* lineDelta, maxDist*MathF.Sqrt2, out arcDelta, out dist) ? float.PositiveInfinity : lineDelta;
        }

        /// <summary>
        /// Tests to see if any points along the biarc are approximately within <paramref name="maxDist"/> of any point along the line
        /// defined by the points <paramref name="start"/> and <paramref name="end"/>. If within range, the relative distance along the
        /// biarc of the closest point, the relative distance along the line, and the distance to that point are outputted.
        /// </summary>
        /// <param name="start">Start point that defines the line to test proximity of</param>
        /// <param name="end">End point that defines the line to test proximity of</param>
        /// <param name="maxDist">Maximum distance between any point of the line and any point of the biarc</param>
        /// <param name="tLine">Relative distance along the line (between 0.0 and 1.0) of the closest point</param>
        /// <param name="tArc">Relative distance along the arc (between 0.0 and 1.0) of the closest point</param>
        /// <param name="dist">Distance between the closest points on the biarc and line</param>
        protected override bool OnIsApproxWithinLineRange(Vector start, Vector end, float maxDist, out float tLine, out float tArc, out float dist)
        {
            tLine = float.NaN;
            tArc = float.NaN;
            dist = float.NaN;

            if (Next == null) return false;
            
            float t0, d0, t1, d1;
            var l0 = ArcLineProximityTest(ref _arc1, ref start, ref end, maxDist, out t0, out d0);
            var l1 = ArcLineProximityTest(ref _arc2, ref start, ref end, maxDist, out t1, out d1);

            if (!float.IsPositiveInfinity(l0) && l0 <= l1)
            {
                tLine = l0;
                tArc = t0;
                dist = d0;
                return true;
            }

            if (!float.IsPositiveInfinity(l1))
            {
                tLine = l1;
                tArc = t1;
                dist = d1;
                return true;
            }

            return false;
        }

        private static bool IsApproxOverlapping(ref Arc a, ref Arc b, float range)
        {
            var dx = a.Center.X - b.Center.X;
            var dz = a.Center.Z - b.Center.Z;

            var dist = a.Radius + b.Radius + range;

            return dx*dx + dz*dz <= dist*dist;
        }

        /// <summary>
        /// Very rough approximation to see if two biarcs may potentially overlap vertically.
        /// </summary>
        /// <param name="other">Other biarc to test for overlaps</param>
        /// <param name="range">Maximum horizontal distance between biarcs for them to overlap</param>
        protected override bool OnIsApproxOverlapping(KeypointCurve other, float range)
        {
            var arc = other as Biarc;
            if (arc == null) return false;

            return IsApproxOverlapping(ref _arc1, ref arc._arc1, range)
                || IsApproxOverlapping(ref _arc2, ref arc._arc1, range)
                || IsApproxOverlapping(ref _arc1, ref arc._arc2, range)
                || IsApproxOverlapping(ref _arc2, ref arc._arc2, range);
        }
    }
}
