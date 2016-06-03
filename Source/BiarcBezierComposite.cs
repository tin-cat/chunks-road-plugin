using System;
using Chunks;
using Chunks.Geometry;
using Chunks.Graphics;

namespace Road.Source
{
    public class BiarcBezierComposite : Biarc
    {
        private struct Bezier
        {
            public Vector P0;
            public Vector P1;
            public Vector P2;
            public Vector P3;

            public void SetKeyPoints(Vector p0, Vector d0, Vector p3, Vector d3)
            {
                P0 = p0;
                P3 = p3;

                var dp = p3 - p0;
                var dt = d3 + d0;

                var a = dt.LengthSquared - 1;
                var b = dp.Dot(dt) * 2f;
                var c = dp.LengthSquared;

                var root = b*b - 4*a*c;

                var tangentScale = 1f;
                if (root >= 0)
                {
                    var solution1 = Math.Abs((-b + MathF.Sqrt(root)) / (2f * a));
                    var solution2 = Math.Abs((-b - MathF.Sqrt(root)) / (2f * a));

                    tangentScale = Math.Min(solution1, solution2);
                }

                P1 = p0 + d0*tangentScale;
                P2 = p3 - d3*tangentScale;
            }

            public Vector GetPosition(float t)
            {
                var s = 1f - t;
                return s * s * s * P0 + 3 * s * s * t * P1 + 3 * s * t * t * P2 + t * t * t * P3;
            }

            public void DrawDebugLines(Color color)
            {
                Debug.DrawLine(P0, P1, color);
                Debug.DrawLine(P1, P2, color);
                Debug.DrawLine(P2, P3, color);
            }
        }

        private Bezier _bezier1;
        private Bezier _bezier2;
        private float _splitT;
        private float _invSplitT;
        private float _invNegSplitT;

        protected override void OnUpdateCurve(out float length)
        {
            base.OnUpdateCurve(out length);

            _splitT = Arc1.ArcLength/length;
            _invSplitT = 1f/_splitT;
            _invNegSplitT = 1f/(1f - _splitT);

            var midPos = base.OnGetPosition(_splitT);
            var midTan = base.OnGetRotation(_splitT)*Vector.UnitZ;

            _bezier1.SetKeyPoints(Start.Position, Start.Tangent, midPos, midTan);
            _bezier2.SetKeyPoints(midPos, midTan, End.Position, End.Tangent);
        }

        protected override Vector OnGetPosition(float t)
        {
            return t < _splitT ? _bezier1.GetPosition(t*_invSplitT) : _bezier2.GetPosition((t - _splitT)*_invNegSplitT);
        }

        protected override Quaternion OnGetRotation(float t)
        {
            const float delta = 1f/128f;

            var baseRot = base.OnGetRotation(t);

            var next = OnGetPosition(Math.Min(1f, t + delta));
            var prev = OnGetPosition(Math.Max(0f, t - delta));

            return Quaternion.LookRotation(next - prev, baseRot*Vector.UnitY);
        }

#if DEBUG
        protected override void OnUpdate()
        {
            base.OnUpdate();

            _bezier1.DrawDebugLines(new Color(1f, 0f, 0f));
            _bezier2.DrawDebugLines(new Color(0f, 1f, 0f));
        }
#endif
    }
}
