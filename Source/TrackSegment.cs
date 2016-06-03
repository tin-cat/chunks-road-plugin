using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Chunks;
using Chunks.Entities;
using Chunks.Geometry;
using Chunks.Graphics;
using Chunks.Plugins;
using Road.Source;

namespace Road
{
    public enum TrackType
    {
        Default,
        Brake,
        ChainLift,
        Booster
    }

    public partial class TrackSegment : Component
    {
        private static readonly Dictionary<int, TrackSegment> _sSegments = new Dictionary<int, TrackSegment>();

        public static IEnumerable<TrackSegment> GetAll()
        {
            return _sSegments.Values.Where(x => x.IsValid);
        }

        private static bool _guidesVisible;
        public static bool GuidesVisible
        {
            get { return _guidesVisible; }
            set
            {
                if (value == _guidesVisible) return;
                _guidesVisible = value;

                foreach (var segment in GetAll())
                {
                    if (segment._guideTransform == null || !segment._guideTransform.IsValid) continue;
                    segment._guideTransform.Entity.GetComponent<ModelRenderer>().Enabled = value;
                }
            }
        }

        public static void UpdateArrowMaterial(TransparentStandardMaterial material, bool highlighted)
        {
            var clr = highlighted ? _sHighlightedColor : _sDefaultColor;

            material.Color = clr;
            material.Emission = clr/16f;
        }

        private static int _sNextUniqueId;

        private TrackSegment _next;
        private ModelRenderer _trackRenderer;
        private KeypointCurve _curve;
        private bool _transformInvalid;
        private bool _trackInvalid;

        private Transform _guideTransform;

        private readonly List<Car> _cars = new List<Car>();
        
        private int _uniqueId = -1;
        private int _nextId = -1;
        private TrackType _trackType = TrackType.Default;

        private bool _highlighted;
        
        private static readonly Color _sDefaultColor = new Color(127, 255, 63, 191) * 0.75f;
        private static readonly Color _sHighlightedColor = new Color(127, 255, 63, 191);

        public Vector Forward => _guideTransform.Forward;
        public Vector Up => _guideTransform.Up;

        public TrackType Type
        {
            get { return _trackType; }
            set
            {
                if (_trackType == value) return;

                _trackType = value;
                InvalidateTrackMesh();
            }
        }

        public bool IsManipulating { get; private set; }

        public bool IsHighlighted
        {
            get { return _highlighted; }
            set
            {
                if (!IsValid) return;
                if (_highlighted == value) return;
                _highlighted = value;

                if (value)
                {
                    foreach (var segment in GetAll())
                    {
                        if (segment == this) continue;
                        segment.IsHighlighted = false;
                    }
                }

                if (_guideTransform == null) return;

                UpdateArrowMaterial((TransparentStandardMaterial) _guideTransform.Entity.GetComponent<ModelRenderer>().Material, IsHighlighted);
            }
        }

        public float Length => _curve.Length;
        
        public IEnumerable<Car> Cars => _cars;

        internal int UniqueId
        {
            get
            {
                if (_uniqueId != -1) return _uniqueId;
                _uniqueId = _sNextUniqueId++;
                _sSegments.Add(_uniqueId, this);
                return _uniqueId;
            }
        }

        public TrackSegment Prev { get; private set; }

        public TrackSegment Next
        {
            get
            {
                if (_nextId == -1) return _next;

                TrackSegment next;
                if (_sSegments.TryGetValue(_nextId, out next)) Next = next;
                _nextId = -1;

                return _next;
            }
            set
            {
                if (_next == value) return;
                if (_next != null) _next.Prev = null;

                foreach (var car in _cars.ToArray())
                {
                    car.DetachFromTrack();
                }

                _next = value;
                _curve.Next = value?._curve;

                if (value != null)
                {
                    if (value.Prev != null) value.Prev.Next = null;

                    value.Prev = this;
                    value.InvalidateTrackMesh();
                }

                InvalidateTrackMesh();
                Prev?.InvalidateTrackMesh();
            }
        }

        internal void AddCar(Car car)
        {
            _cars.Add(car);
        }

        internal void RemoveCar(Car car)
        {
            _cars.Remove(car);
        }

        public Car FindNextCar(float from)
        {
            return FindNextCar(this, from);
        }

        private Car FindNextCar(TrackSegment first, float from = float.NegativeInfinity)
        {
            if (float.IsNegativeInfinity(from) && first == this) return null;

            return _cars.OrderBy(x => x.TrackSegmentProgress)
                .FirstOrDefault(x => x.TrackSegmentProgress > from)
                   ?? Next?.FindNextCar(first);
        }

        public Car FindPrevCar(float from)
        {
            return FindPrevCar(this, from);
        }

        private Car FindPrevCar(TrackSegment first, float from = float.PositiveInfinity)
        {
            if (float.IsPositiveInfinity(from) && first == this) return null;

            return _cars.OrderByDescending(x => x.TrackSegmentProgress)
                .FirstOrDefault(x => x.TrackSegmentProgress < from)
                   ?? Prev?.FindPrevCar(first);
        }

        protected override void OnInitialize()
        {
            Name = "Track";
            Entity.ShouldSave = true;

            _trackRenderer = AddComponent<ModelRenderer>();
            _trackRenderer.Model = new Model();
            _trackRenderer.Model.SetBounds(Vector.One*-16f, Vector.One*16f);

            _trackRenderer.Material = BlockMaterial.Default;

            _guideTransform = CreateChild().Transform;
            _guideTransform.Entity.Name = "Guide";
            _guideTransform.LocalScale = Vector.One/World.ChunkSize;

            _curve = _guideTransform.AddComponent<Biarc>();
            _curve.Invalidate();
            
            var renderer = _guideTransform.AddComponent<ModelRenderer>();
            renderer.Model = Plugin.GetResource<Model>("Road.Arrow");
            renderer.Material = new TransparentStandardMaterial();
            renderer.Enabled = GuidesVisible;

            UpdateArrowMaterial((TransparentStandardMaterial) renderer.Material, IsHighlighted);
        }

        protected override void OnDestroy()
        {
            if (IsManipulating)
            {
                _guideTransform.Entity.Destroy();
            }

            Next = null;
            if (Prev != null) Prev.Next = null;

            if (_uniqueId == -1) return;

            Debug.Assert(_sSegments[_uniqueId] == this);

            _sSegments.Remove(_uniqueId);
        }

        public TrackSegment CreateFromEnd()
        {
            if (Next != null && Prev != null) return null;

            var inst = World.CreateEntity().AddComponent<TrackSegment>();

            inst.Transform.Position = Transform.Position;
            inst._guideTransform.Rotation = _guideTransform.Rotation;

            if (Next == null) Next = inst;
            else inst.Next = this;

            inst.UpdateTransform();

            return inst;
        }

        public void InvalidateTransform()
        {
            _transformInvalid = true;
        }

        public void UpdateTransform()
        {
            // Force ID assignment
            // ReSharper disable once UnusedVariable
            var id = UniqueId;

            _transformInvalid = false;
            
            Transform.Position = _guideTransform.Position;

            if (!IsManipulating) {
                var rot = _guideTransform.Rotation;
                _guideTransform.LocalPosition = Vector.Zero;
                Transform.Rotation = Quaternion.Identity;
                _guideTransform.LocalRotation = rot;
            }

            Invalidate();
        }

        private void Invalidate()
        {
            InvalidateTrackMesh();
            Prev?.InvalidateTrackMesh();
        }

        public Transform StartManipulating()
        {
            if (IsManipulating) throw new Exception("Already manipulating track!");
            IsManipulating = true;

            _guideTransform.DetachFromParent(true);
            return _guideTransform;
        }

        public void StopManipulating()
        {
            if (!IsManipulating) throw new Exception("Not manipulating track!");
            IsManipulating = false;

            Transform.Add(_guideTransform, true);

            UpdateTransform();
        }

        public Vector GetTrackPos(float t)
        {
            return _curve.GetPosition(t);
        }

        public Vector GetTrackPosAtDelta(float delta)
        {
            return _curve.GetPosition(delta* _curve.InverseLength);
        }

        public bool IsWithinRange(Vector pos, float maxRange, out float t)
        {
            float dist;
            return _curve.IsWithinRange(pos, maxRange, out t, out dist);
        }

        public bool IsApproximatelyInLineRange(Vector start, Vector end, float maxRange, out float lineDelta, out float trackDelta)
        {
            float dist;
            return _curve.IsApproxWithinLineRange(start, end, maxRange, out lineDelta, out trackDelta, out dist);
        }

        public Quaternion GetTrackRotation(float t)
        {
            return _curve.GetRotation(t);
        }

        public void GetTrackTransform(float t, out Vector position, out Quaternion rotation)
        {
            if (Next == null || t < 0f && Prev == null)
            {
                position = _guideTransform.Position;
                rotation = _guideTransform.Rotation;
                return;
            }

            position = GetTrackPos(t);
            rotation = GetTrackRotation(t);
        }

        public void GetTrackTransformAtDelta(float delta, out Vector position, out Quaternion rotation)
        {
            GetTrackTransform(delta* _curve.InverseLength, out position, out rotation);
        }

        public float GetTrackGradientAtDelta(float delta)
        {
            return (GetTrackRotation(delta* _curve.InverseLength)*Vector.UnitZ).Y;
        }

        protected override void OnSave(BinaryWriter writer)
        {
            writer.Write(UniqueId);
            writer.Write(Next?.UniqueId ?? -1);
            writer.Write(_guideTransform.LocalRotation);

            writer.Write((byte) _trackType);

            // Reserved for later
            writer.Write((byte) 0x00);
            writer.Write((short) 0x00);
        }

        protected override void OnLoad(BinaryReader reader)
        {
            _uniqueId = reader.ReadInt32();
            _nextId = reader.ReadInt32();

            _sSegments.Add(_uniqueId, this);

            if (_sNextUniqueId <= _uniqueId) _sNextUniqueId = _uniqueId + 1;

            _guideTransform.LocalRotation = reader.ReadQuaternion();

            _trackType = (TrackType) reader.ReadByte();

            reader.ReadByte();
            reader.ReadInt16();

            UpdateTransform();
        }
    }
}
