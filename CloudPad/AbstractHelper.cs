using OpenTK;

namespace _3d
{
    public abstract class AbstractHelper
    {
        public abstract void Draw();
        public bool Visible { get; set; } = true;
        
        public Vector3d[] RelatedPoints
        {
            get
            {
                return RelatedIndexer.GetPoints(RelatedIndexer.Indicies);
            }
        }
        public PointIndexer RelatedIndexer;
        public int RelatedPointsCount
        {
            get
            {
                if (RelatedPoints == null) return 0;
                return RelatedPoints.Length;
            }
        }
    }
}

