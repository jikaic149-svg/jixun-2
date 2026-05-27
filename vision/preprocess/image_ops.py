def safe_copy(img):
    try:
        return img.copy()
    except Exception:
        return img
