def lineTrackingStatus(px):
    a = px.get_grayscale_data()
    t = 800
    return [a[0]>t, a[1]>t, a[2]>t]
