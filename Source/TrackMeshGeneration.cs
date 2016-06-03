using System;
using System.Collections.Generic;
using Chunks;
using Chunks.Geometry;
using Chunks.Graphics;
using Chunks.Plugins;

namespace Road
{
    partial class TrackSegment
    {
        private static readonly ModelGenerator _sMeshGenerator = new ModelGenerator(1f, PrimitiveType.Quads);

        private readonly Surface _supportSurf = Plugin.GetResource<Surface>("Road.Support");

        private readonly Dictionary<TrackType, Surface> _trackSurfaces = new Dictionary<TrackType, Surface>
        {
            { TrackType.Default, Plugin.GetResource<Surface>($"Road.Track.{TrackType.Default}") },
            { TrackType.Brake, Plugin.GetResource<Surface>($"Road.Track.{TrackType.Brake}") },
            { TrackType.ChainLift, Plugin.GetResource<Surface>($"Road.Track.{TrackType.ChainLift}") },
            { TrackType.Booster, Plugin.GetResource<Surface>($"Road.Track.{TrackType.Booster}") }
        }; 
        
        private readonly List<TrackSegment> _potentialCrossovers = new List<TrackSegment>();
        
        private bool CouldCrossOver(TrackSegment other, float range)
        {
            return _curve.IsApproxOverlapping(other._curve, range);
        }

        private void UpdateTrackMesh()
        {
            _trackInvalid = false;

            // ReSharper disable once UnusedVariable
            var id = UniqueId;

            if (_guideTransform != null)
            {
                _guideTransform.LocalScale = Vector.One/World.ChunkSize;
            }

            _sMeshGenerator.Clear();

            if (Next != null)
            {
                var halfWidth = (.5f/World.ChunkSize);
                var supportWidth = halfWidth*(5f/8f);

                int pttl = 0, pttr = 0, ptbl = 0, ptbr = 0;
                int pstl = 0, pstr = 0, psbl = 0, psbr = 0;

                var first = true;
                var prevSupported = false;

                var trackTex = 0f;
                var supportTex = 0f;

                var hitRange = 2f/World.ChunkSize;

                var prevGroundPos = Vector.Zero;

                _potentialCrossovers.Clear();
                foreach (var trackSegment in GetAll())
                {
                    if (trackSegment == this) continue;
                    if (trackSegment.Next == null) continue;

                    if (trackSegment.CouldCrossOver(this, hitRange))
                    {
                        _potentialCrossovers.Add(trackSegment);
                    }
                }

                var tScale = MathF.Ceiling(_curve.Length * World.ChunkSize);
                var start = _curve.GetPosition(0f);

                var minDist = (1f/4f)/World.ChunkSize;
                var maxDist = 4f/World.ChunkSize;

                foreach (var tRaw in _curve.GetDeltas(MathF.Pi / 32f, minDist, maxDist))
                {
                    var t = MathF.Clamp01(tRaw);

                    Vector pos;
                    Quaternion rot;
                    GetTrackTransform(t, out pos, out rot);

                    var right = rot*Vector.UnitX;
                    var up = rot*Vector.UnitY;

                    var groundPos = pos*new Vector(1f, 0f, 1f);

                    if (!first)
                    {
                        var groundDiff = (groundPos - prevGroundPos).Length;

                        supportTex += groundDiff*World.ChunkSize;
                        trackTex = t * tScale;
                    }

                    prevGroundPos = groundPos;

                    var relPos = pos - start;
                    var surf = _trackSurfaces[_trackType];

                    var ttl = _sMeshGenerator.AddVertex(relPos - right*halfWidth, new Vector(trackTex, 0f), up,
                        Color.White, surf);
                    var ttr = _sMeshGenerator.AddVertex(relPos + right*halfWidth, new Vector(trackTex, 1f), up,
                        Color.White, surf);
                    var tbl = _sMeshGenerator.AddVertex(relPos - right*halfWidth, new Vector(trackTex, 0f), -up,
                        Color.White, surf);
                    var tbr = _sMeshGenerator.AddVertex(relPos + right*halfWidth, new Vector(trackTex, 1f), -up,
                        Color.White, surf);

                    var supports = Math.Abs(right.Dot(Vector.UnitY)) < 0.866 && up.Dot(Vector.UnitY) > 0f;

                    var ground = 0f;

                    if (supports)
                    {
                        var ray = new Ray(pos, -Vector.UnitY);
                        WorldRaycastHit hit;
                        if (World.RaycastWorld(ray, 4f, out hit))
                        {
                            ground = hit.Position.Y;
                        }

                        var hitTestStart = pos - Vector.UnitY*hitRange*1.5f;

                        foreach (var segment in _potentialCrossovers)
                        {
                            float arcDelta, lineDelta;
                            if (!segment.IsApproximatelyInLineRange(hitTestStart,
                                new Vector(hitTestStart.X, ground, hitTestStart.Z), hitRange, out lineDelta, out arcDelta)) continue;

                            supports = false;
                            break;
                        }
                    }

                    if (supports)
                    {
                        var sl = pos - right*supportWidth;
                        var sr = pos + right*supportWidth;

                        var stl = _sMeshGenerator.AddVertex(sl - start, new Vector(supportTex, sl.Y * 16f), -right,
                            Color.White, _supportSurf);
                        var str = _sMeshGenerator.AddVertex(sr - start, new Vector(supportTex + 1f, sr.Y * 16f), right,
                            Color.White, _supportSurf);

                        sl -= right*(sl.Y - ground)/16f;
                        sr += right*(sr.Y - ground)/16f;

                        sl = sl*new Vector(1f, 0f, 1f) + Vector.UnitY*ground;
                        sr = sr*new Vector(1f, 0f, 1f) + Vector.UnitY*ground;

                        var sbl = _sMeshGenerator.AddVertex(sl - start, new Vector(supportTex, ground * 16f), -right,
                            Color.White, _supportSurf);
                        var sbr = _sMeshGenerator.AddVertex(sr - start, new Vector(supportTex + 1f, ground * 16f),
                            right, Color.White, _supportSurf);

                        if (prevSupported)
                        {
                            _sMeshGenerator.AddQuadIndices(stl, pstl, psbl, sbl);
                            _sMeshGenerator.AddQuadIndices(sbr, psbr, pstr, str);
                        }
                        else
                        {
                            _sMeshGenerator.AddQuadIndices(stl, str, sbr, sbl);
                        }

                        pstl = stl;
                        pstr = str;
                        psbl = sbl;
                        psbr = sbr;

                        prevSupported = true;
                    }
                    else if (prevSupported)
                    {
                        _sMeshGenerator.AddQuadIndices(pstr, pstl, psbl, psbr);

                        prevSupported = false;
                    }

                    if (!first)
                    {
                        _sMeshGenerator.AddQuadIndices(ttr, pttr, pttl, ttl);
                        _sMeshGenerator.AddQuadIndices(tbl, ptbl, ptbr, tbr);
                    }
                    else first = false;

                    pttl = ttl;
                    pttr = ttr;
                    ptbl = tbl;
                    ptbr = tbr;
                }

                if (prevSupported)
                {
                    _sMeshGenerator.AddQuadIndices(pstr, pstl, psbl, psbr);
                }
            }

            _trackRenderer.Model.Update(_sMeshGenerator);
        }

        public void InvalidateTrackMesh()
        {
            _trackInvalid = true;
            _curve?.Invalidate();
        }

        protected override void OnPreRender()
        {
            if (_transformInvalid) UpdateTransform();
            if (_trackInvalid) UpdateTrackMesh();
        }
    }
}