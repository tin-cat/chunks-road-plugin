using System;
using System.Linq;
using Chunks.Geometry;
using Chunks.Interface;

namespace Road
{
    public abstract class RoadTool : WandTool
    {
        protected Car HoveredCar { get; private set; }
        protected Car RiddenCar { get; private set; }
        
        protected float SelectionRange => Math.Max(2f * Wand.Transform.LossyScale.X, 0.5f) / World.ChunkSize;

        protected virtual bool CanRideCar
        {
            get { return !Wand.HairTrigger.IsHeld && !Wand.Grip.IsHeld; }
        }

        protected TrackSegment FindClosest(Vector pos)
        {
            return FindClosest(pos, SelectionRange, x => !x.IsManipulating);
        }

        protected TrackSegment FindClosest(Vector pos, float maxDist)
        {
            return FindClosest(pos, maxDist, x => !x.IsManipulating);
        }

        protected TrackSegment FindClosest(Vector pos, float maxDist, Predicate<TrackSegment> condition)
        {
            var rangeSquared = maxDist * maxDist;
            return TrackSegment.GetAll()
                .Where(x => condition(x) && (x.Transform.Position - pos).LengthSquared <= rangeSquared)
                .OrderBy(x => (x.Transform.Position - pos).LengthSquared)
                .FirstOrDefault();
        }

        protected override void OnUpdate()
        {
            base.OnUpdate();
            
            var pos = Wand.GetCursorPosition();
            var rangeSquared = SelectionRange * SelectionRange;

            HoveredCar = World.GetRootComponents<Car>()
                .Where(x => x.GetDistanceSquared(pos) <= rangeSquared)
                .OrderBy(x => x.GetDistanceSquared(pos))
                .FirstOrDefault();

            RiddenCar = World.CameraRig.Transform.Parent?.Entity.GetComponent<Car>();

            if (CanRideCar && HoveredCar != null && RiddenCar == null)
            {
                Wand.Pointer.IsVisible = true;
                Wand.Pointer.Position = HoveredCar.Transform.Position;
            }
        }

        protected bool TryRideClosestCar()
        {
            if (RiddenCar != null)
            {
                RiddenCar.StopRiding();
                return true;
            }

            if (HoveredCar != null)
            {
                HoveredCar.StartRiding();
                return true;
            }

            return false;
        }
    }
}
