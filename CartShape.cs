using UnityEngine;

namespace CartPilot;

/// <summary>Cart footprint measured from its own solid colliders, in cart-local space.</summary>
internal struct CartShape
{
    public float HalfWidth;   // local X
    public float HalfLength;  // local Z
    public float Height;
    public float BottomY;     // local Y of the lowest collider point

    public static CartShape Measure(Rigidbody rb)
    {
        var root = rb.transform;
        var local = new Bounds();
        bool any = false;

        foreach (var col in rb.GetComponentsInChildren<Collider>())
        {
            if (col.isTrigger || col.attachedRigidbody != rb || !TryOwnBounds(col, out var b)) continue;

            // 8 corners: collider space -> world -> cart space
            for (int i = 0; i < 8; i++)
            {
                var c = new Vector3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
                var p = root.InverseTransformPoint(col.transform.TransformPoint(Vector3.Scale(c, b.size) + b.min));
                if (!any) { local = new Bounds(p, Vector3.zero); any = true; }
                else local.Encapsulate(p);
            }
        }

        if (!any)
        {
            Plugin.Log.LogWarning("Cart has no measurable colliders, using fallback size.");
            return new CartShape { HalfWidth = 0.6f, HalfLength = 0.9f, Height = 1f, BottomY = 0f };
        }

        // Transform scale is already applied by TransformPoint; local is in cart units, so scale back to meters.
        var s = root.lossyScale;
        return new CartShape
        {
            HalfWidth = Mathf.Max(Mathf.Abs(local.min.x), Mathf.Abs(local.max.x)) * s.x,
            HalfLength = Mathf.Max(Mathf.Abs(local.min.z), Mathf.Abs(local.max.z)) * s.z,
            Height = local.size.y * s.y,
            BottomY = local.min.y * s.y,
        };
    }

    private static bool TryOwnBounds(Collider col, out Bounds b)
    {
        switch (col)
        {
            case BoxCollider box: b = new Bounds(box.center, box.size); return true;
            case SphereCollider sph: b = new Bounds(sph.center, Vector3.one * sph.radius * 2f); return true;
            case CapsuleCollider cap:
                var size = Vector3.one * cap.radius * 2f;
                size[cap.direction] = Mathf.Max(cap.height, cap.radius * 2f);
                b = new Bounds(cap.center, size);
                return true;
            case MeshCollider mesh when mesh.sharedMesh != null: b = mesh.sharedMesh.bounds; return true;
            default: b = default; return false;
        }
    }

    public override string ToString() =>
        $"halfWidth={HalfWidth:F2} halfLength={HalfLength:F2} height={Height:F2} bottomY={BottomY:F2}";
}
