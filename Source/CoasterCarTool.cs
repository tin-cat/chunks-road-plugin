using System;
using System.Diagnostics;
using System.Linq;
using Chunks;
using Chunks.Geometry;
using Chunks.Graphics;
using Chunks.Interface;

namespace Road
{
    [WandToolMenuItem("Road Car Control",
        Category = "Road",
        Description = "Create and remote control road cars.",
        IconName = "Road.ControllerIcon",
        Order = 6)]
    public class RoadCarTool : RoadTool
    {
        private readonly Stopwatch _timer = new Stopwatch();
        private TimeSpan _nextPulse;
        private Car _controlled;

        private TrackSegment _closestTrack;
        private float _closestDelta;

        public override Color Color => new Color(0f, .65f, .9f);

        protected override void OnInitialize()
        {
            base.OnInitialize();

            Wand.HairTrigger.Pressed += OnHairTriggerPressed;
            Wand.TrackPad.Pressed += OnTrackPadPressed;
        }

        private void UpdateClosestTrack()
        {
            _closestTrack = null;
            _closestDelta = float.NaN;

            var closestDist = float.MaxValue;
            var pos = Wand.GetCursorPosition();

            foreach (var segment in TrackSegment.GetAll())
            {
                if (segment.Next == null) continue;

                float t;
                if (!segment.IsWithinRange(pos, SelectionRange, out t)) continue;

                var trackPos = segment.GetTrackPos(t);
                var dist = (trackPos - pos).LengthSquared;

                if (dist >= closestDist) continue;

                _closestTrack = segment;
                _closestDelta = t;
                closestDist = dist;
            }
        }

        private void OnTrackPadPressed(object sender, WandButton.EventArgs e)
        {
            if (TryRideClosestCar()) return;
            
            if (_closestTrack?.Next == null) return;

            var prev = World.CreateEntity()
                .AddComponent<Car>();

            prev.Name = "Head";
            prev.AttachToTrack(_closestTrack, _closestDelta * _closestTrack.Length);

            /*
             * Only one car, this is no coaster!
            for (var i = 0; i < 3; ++i)
            {
                var next = World.CreateEntity()
                    .AddComponent<Car>();

                next.Name = $"Car {i + 1}";
                next.AttachBehindCar(prev);

                if (next.CurrentTrackSegment == null) break;

                prev = next;
            }
            */
        }

        protected override void OnActivate()
        {
            base.OnActivate();
            
            Wand.Pointer.IsVisible = false;

            Wand.TrackPadToolTip.Text = "Spawn a car on track / ride existing car";

            _timer.Reset();
            _timer.Start();

            _nextPulse = TimeSpan.Zero;

            StopControlling();
        }

        private void OnHairTriggerPressed(object sender, WandButton.EventArgs e)
        {
            if (_controlled != null) return;

            var pos = Wand.GetCursorPosition();

            var car = World.GetRootComponents<Car>()
                .Where(x => !x.AttachedToNextCar)
                .OrderBy(x => (x.Transform.Position - pos).LengthSquared)
                .FirstOrDefault();

            if (car != null) StartControlling(car);
        }

        private void StartControlling(Car car)
        {
            _controlled = car;

            Wand.TriggerToolTip.Text = "Hold to accelerate";
            Wand.GripToolTip.Text = "Hold to brake";
        }

        private void StopControlling()
        {
            if (_controlled != null)
            {
                _controlled.Torque = 0f;
                _controlled.Friction = Car.DefaultFriction;
            }

            _controlled = null;

            Wand.TriggerToolTip.Text = "Press near a car to start controlling";
            Wand.GripToolTip.Text = "";
        }

        protected override void OnDeactivate()
        {
            StopControlling();
        }

        protected override void OnUpdate()
        {
            base.OnUpdate();

            UpdateClosestTrack();

            if (HoveredCar == null)
            {
                Wand.Pointer.IsVisible = _closestTrack != null;
                Wand.Pointer.Position = _closestTrack != null ? _closestTrack.GetTrackPos(_closestDelta) : Vector.Zero;
            }

            if (_controlled != null && !_controlled.IsValid) StopControlling();
            if (_controlled == null) return;

            Wand.Pointer.IsVisible = true;
            Wand.Pointer.Position = _controlled.Transform.Position;

            if (Wand.Grip.IsHeld)
            {
                _controlled.Friction = 0.875f;
                return;
            }

            _controlled.Friction = 0.25f;

            var value = MathF.Clamp01(Wand.TriggerValue*2f);

            _controlled.Torque = value;

            if (_timer.Elapsed < _nextPulse || value <= 0f) return;

            Wand.Particles.Emit(1, .5f + value);
            Wand.TriggerHapticPulse(250 + (int) (value * 250));

            _nextPulse = _timer.Elapsed + TimeSpan.FromSeconds(1f / 45f + (1f - value) * .25f);
        }
    }
}
