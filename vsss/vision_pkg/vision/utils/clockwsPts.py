def clockwise_pts(pts):
    """
    Sorts points in a clockwise order starting from the top-left.
    
    Args:
        pts (ndarray): Array of points to sort.
    
    Returns:
        ndarray: Points sorted in clockwise order [top-left, top-right, bottom-right, bottom-left].
    """
    xSort = pts[np.argsort(pts[:,0]), :]
    #select the first two rows and all the colums of it.
    leftMost = xSort[:2, :] 
    rightMost = xSort[2:, :]
    
    #sort them by their Y coordinate
    leftmost = leftMost[np.argsort(leftMost[:, 1]), :] 
    tl, bl = leftmost
    #distances from tl to both points of rightmost
    right = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0] 
    br, tr = rightMost[np.argsort(right)[::-1], :]

    return np.array([tl, tr, br, bl])