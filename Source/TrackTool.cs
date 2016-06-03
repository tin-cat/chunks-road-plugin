using System;
using Chunks.Entities;
using Chunks.Geometry;
using Chunks.Graphics;
using Chunks.Interface;
using Chunks.Plugins;

namespace Road
{
    [WandToolMenuItem("Road Track Tool",
        Category = "Road",
        Description = "Draw and modify road tracks.",
        IconName = "Road.RoadIcon",
        Order = 5)]
    public class TrackTool : RoadTool
    {
        private TrackSegment _hoveredNode;
        private TrackSegment _hoveredSegment;
        private float _hoveredSegmentT;
        private TrackSegment _startSegment;
        private TrackSegment _selected;
        private Transform _selectedGuide;
        private ModelRenderer _arrowRenderer;

        private float _lastHoveredUpdate;
        private Vector _lastPulsePos;
        private TrackType _lastTrackType;
        
        private float PulseDistance => Wand.Transform.LossyScale.X/World.ChunkSize;

        private float NewTrackThreshold => Wand.Transform.LossyScale.X*.5f/World.ChunkSize;
        
        public override Color Color => new Color(1f, .65f, 0f);
        
        protected override void OnInitialize()
        {
            Wand.HairTrigger.Pressed += OnTriggerPressed;
            Wand.HairTrigger.Released += OnTriggerReleased;

            Wand.Grip.Pressed += OnGripPressed;

            Wand.TrackPad.Pressed += OnTrackPadPressed;

            base.OnInitialize();
        }

        protected override void OnActivate()
        {
            base.OnActivate();
            
            Wand.Pointer.IsVisible = true;

            Wand.TriggerToolTip.Text = "Draw / move track segments";
            Wand.GripToolTip.Text = "Remove track segments";
            Wand.TrackPadToolTip.Text = "Press to toggle track type";

            TrackSegment.GuidesVisible = true;

            _arrowRenderer = World.CreateEntity().AddComponent<ModelRenderer>();
            _arrowRenderer.Model = Plugin.GetResource<Model>("Road.Arrow");
            _arrowRenderer.Material = new TransparentStandardMaterial();
            _arrowRenderer.Entity.Enabled = false;
            _arrowRenderer.Transform.LocalScale = Vector.One/World.ChunkSize;

            TrackSegment.UpdateArrowMaterial((TransparentStandardMaterial) _arrowRenderer.Material, true);
        }

        private void FindHoveredNode()
        {
            // ReSharper disable once CompareOfFloatsByEqualityOperator
            if (World.Time == _lastHoveredUpdate) return;
            _lastHoveredUpdate = World.Time;

            var pos = Wand.GetCursorPosition();
            if (HoveredCar == null) Wand.Pointer.Position = pos;

            var prevHovered = _hoveredSegment;
            _hoveredSegment = null;
            
            if (_selected != null) return;

            var hovered = FindClosest(pos);

            if (_hoveredNode != hovered)
            {
                if (hovered == null)
                {
                    _hoveredNode.IsHighlighted = false;
                }

                _hoveredNode = hovered;

                if (_hoveredNode != null)
                {
                    _hoveredNode.IsHighlighted = true;
                    Wand.HapticFeedback(HapticFeedbackType.Tick);
                }
            }

            if (_hoveredNode != null)
            {
                if (_hoveredNode.Next != null && _hoveredNode.Prev != null)
                {
                    if (HoveredCar == null) Wand.Pointer.Position = _hoveredNode.Transform.Position;
                    return;
                }

                var forward = _hoveredNode.Forward;
                var diff = pos - _hoveredNode.Transform.Position;
                var dist = forward.Dot(diff);
                var threshold = NewTrackThreshold;

                if (_hoveredNode.Next == null && dist < threshold || _hoveredNode.Prev == null && dist > -threshold)
                {
                    if (HoveredCar == null) Wand.Pointer.Position = _hoveredNode.Transform.Position;
                    return;
                }

                _arrowRenderer.Entity.Enabled = true;
                Wand.Pointer.Position = _arrowRenderer.Transform.Position = _hoveredNode.Transform.Position + forward * dist;
                _arrowRenderer.Transform.Rotation = Quaternion.LookRotation(forward, _hoveredNode.Up);

                return;
            }

            var closest = float.MaxValue;
            foreach (var trackSegment in TrackSegment.GetAll())
            {
                if (trackSegment.Next == null) continue;

                float t;
                if (!trackSegment.IsWithinRange(pos, SelectionRange, out t)) continue;

                var trackPos = trackSegment.GetTrackPos(t);
                var dist = (trackPos - pos).LengthSquared;

                if (dist >= closest) continue;

                if (HoveredCar == null) Wand.Pointer.Position = trackPos;
                closest = dist;

                _hoveredSegment = trackSegment;
                _hoveredSegmentT = t;
            }

            if (prevHovered == _hoveredSegment) return;

            Wand.HapticFeedback(HapticFeedbackType.Tick);
        }

        protected override bool CanInteractWithMenuPanel(MenuPanel panel)
        {
            if (_selected != null || !base.CanInteractWithMenuPanel(panel)) return false;

            FindHoveredNode();

            return _hoveredNode == null && _hoveredSegment == null;
        }

        protected override void OnUpdate()
        {
            base.OnUpdate();

            _arrowRenderer.Entity.Enabled = false;

            if (RiddenCar != null)
            {
                if (_selected != null) ReleaseSelected();
                Wand.Pointer.IsVisible = false;
                return;
            }

            Wand.Pointer.IsVisible = true;

            if (_selected == null)
            {
                FindHoveredNode();
                return;
            }

            if (!_selected.IsValid)
            {
                _selected = null;
                return;
            }

            if (HoveredCar == null) Wand.Pointer.Position = _selected.Transform.Position;

            if ((_selectedGuide.Position - _lastPulsePos).Length > PulseDistance)
            {
                Wand.HapticFeedback(HapticFeedbackType.Tick);
                _lastPulsePos = _selectedGuide.Position;
            }

            if (_startSegment != null)
            {
                var diff = _selected.Transform.Position - _startSegment.Transform.Position;
                var rot = Quaternion.LookRotation(diff, Vector.UnitY);

                var trans = _startSegment.StartManipulating();
                trans.Rotation = rot;
                _startSegment.StopManipulating();

                _selectedGuide.Rotation = rot;
            }

            var connectable = FindConnectable();
            if (connectable != null) connectable.IsHighlighted = true;
            else _selected.IsHighlighted = true;

            _selected.InvalidateTransform();
        }
        
        private void OnTriggerPressed(object sender, WandButton.EventArgs e)
        {
            if (RiddenCar != null) return;

            var snapToCursorPos = false;

            if (_hoveredSegment != null)
            {
                Vector pos;
                Quaternion rot;
                _hoveredSegment.GetTrackTransform(_hoveredSegmentT, out pos, out rot);

                var oldNext = _hoveredSegment.Next;
                _hoveredSegment.Next = null;

                _selected = _hoveredSegment.CreateFromEnd();

                var trans = _selected.StartManipulating();
                trans.Position = pos;
                trans.Rotation = rot;
                _selected.StopManipulating();

                _selected.Next = oldNext;
                _selected.Type = _hoveredSegment.Type;
            }
            else if (_hoveredNode != null)
            {
                _selected = _hoveredNode;

                var diff = Wand.GetCursorPosition() - _selected.Transform.Position;
                var forward = _selected.Forward;
                var threshold = NewTrackThreshold;

                if (_selected.Next == null && diff.Dot(forward) > threshold
                    || _selected.Prev == null && diff.Dot(forward) < -threshold)
                {
                    _selected = _selected.CreateFromEnd() ?? _selected;
                    _selected.Type = _hoveredNode.Type;
                    snapToCursorPos = true;
                }
            }
            else
            {
                _startSegment = World.CreateEntity().AddComponent<TrackSegment>();
                _selected = World.CreateEntity().AddComponent<TrackSegment>();

                _startSegment.Next = _selected;

                var pos = Wand.GetCursorPosition();

                var trans = _startSegment.StartManipulating();
                trans.Position = pos;
                _startSegment.StopManipulating();

                trans = _selected.StartManipulating();
                trans.Position = pos;
                _selected.StopManipulating();
            }

            _selected.IsHighlighted = true;

            Wand.Particles.Emit(8);
            Wand.Transform.Add(_selectedGuide = _selected.StartManipulating(), true);

            if (snapToCursorPos)
            {
                _selectedGuide.Position = Wand.GetCursorPosition();
            }
        }

        private void OnTrackPadPressed(object sender, WandButton.EventArgs e)
        {
            if (TryRideClosestCar()) return;

            var segment = _hoveredNode ?? _hoveredSegment;
            if (segment == null) return;

            if (segment.Type == _lastTrackType)
            {
                _lastTrackType = (TrackType) ((int) _lastTrackType + 1);

                if (!Enum.IsDefined(typeof (TrackType), _lastTrackType))
                {
                    _lastTrackType = TrackType.Default;
                }
            }

            segment.Type = _lastTrackType;
        }

        private TrackSegment FindConnectable()
        {
            var next = _selected.Next == null;
            var prev = _selected.Prev == null;
            return FindClosest(_selected.Transform.Position, SelectionRange,
                x => next && x.Prev == null || prev && x.Next == null || _selected.Next == x || _selected.Prev == x);
        }

        private void ReleaseSelected()
        {
            if (_selected == null) return;

            _selected.StopManipulating();

            var connectable = FindConnectable();
            if (connectable != null)
            {
                if (_selected.Next == connectable && _selected.Prev == null
                    || _selected.Prev == connectable && _selected.Next == null)
                {
                    connectable.DestroyEntity();
                }
                else if (_selected.Next == null || _selected.Next == connectable)
                {
                    _selected.Prev.Next = connectable;
                }
                else if (_selected.Prev == null || _selected.Prev == connectable)
                {
                    connectable.Next = _selected.Next;
                }

                _selected.DestroyEntity();
            }

            _selected = null;
            _startSegment = null;
        }

        private void OnTriggerReleased(object sender, WandButton.EventArgs e)
        {
            if (RiddenCar != null) return;
            ReleaseSelected();
        }

        private void OnGripPressed(object sender, WandButton.EventArgs e)
        {
            if (RiddenCar != null) return;
            if (_selected != null) return;

            if (_hoveredSegment != null)
            {
                if (_hoveredSegment.Next.Next == null)
                {
                    _hoveredSegment.Next.DestroyEntity();
                }

                if (_hoveredSegment.Prev == null)
                {
                    _hoveredSegment.DestroyEntity();
                    return;
                }

                _hoveredSegment.Next = null;
                return;
            }

            if (_hoveredNode == null) return;

            if (_hoveredNode.Prev != null && _hoveredNode.Next != null)
            {
                if (_hoveredNode.Prev == _hoveredNode.Next)
                {
                    _hoveredNode.Next.DestroyEntity();
                }
                else
                {
                    _hoveredNode.Prev.Next = _hoveredNode.Next;
                }
            }
            else if (_hoveredNode.Next != null && _hoveredNode.Next.Next == null)
            {
                _hoveredNode.Next.DestroyEntity();
            }
            else if (_hoveredNode.Prev != null && _hoveredNode.Prev.Prev == null)
            {
                _hoveredNode.Prev.DestroyEntity();
            }

            _hoveredNode.DestroyEntity();
        }

        protected override void OnDeactivate()
        {
            TrackSegment.GuidesVisible = false;

            OnTriggerReleased(null, null);
        }
    }
}
