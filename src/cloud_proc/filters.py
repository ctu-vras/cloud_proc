from __future__ import absolute_import, division, print_function
import numpy as np
from ros_numpy import numpify
import rospy
from timeit import default_timer as timer
from tf2_client import get_buffer
from tf2_py import TransformException
from .utils import col, resolve_class

__all__ = [
    'Box',
    'configure_filters',
    'DiscardOld',
    'Filter',
    'FilterChain',
    'KeepFields',
    'Transform'
]


def configure_filters(configs=()):
    assert isinstance(configs, (list, tuple))
    filters = []
    for c in configs:
        assert isinstance(c, dict) and len(c) == 1 and isinstance(c[1], dict)
        name, kwargs = c.items()[0]
        cls = resolve_class(name)
        f = cls(**kwargs)
        filters.append(f)
    return filters


class Filter(object):

    def filter(self, cloud, header):
        return cloud, header

    def __call__(self, cloud, header):
        return filter(cloud, header)

    def __str__(self):
        return '%s' % self.__class__


class PassThrough(Filter):
    pass


class Transform(Filter):

    def __init__(self, target_frame=None, transform=(('x', 'y', 'z'),), rotate=(), timeout=0.0, update_frame=True):
        self.tf = get_buffer()
        self.target_frame = target_frame
        self.timeout = timeout
        self.transform = transform
        self.rotate = rotate
        self.update_frame = update_frame

    def filter(self, cloud, header):
        try:
            tf = self.tf.lookup_transform(self.target_frame, header.frame_id, header.stamp, self.timeout)
        except TransformException as ex:
            rospy.logwarn(ex.message)
            return None, None

        T = numpify(tf.transform)
        R, t = T[:3, :3], T[:3, 3:]
        for fields in self.transform:
            x = np.stack(cloud[f] for f in fields)
            x = np.matmul(R, x) + t
            for i, f in enumerate(fields):
                cloud[f] = x[i, :]
        for fields in self.rotate:
            x = np.stack(cloud[f] for f in fields)
            x = np.matmul(R, x)
            for i, f in enumerate(fields):
                cloud[f] = x[i, :]
        if self.update_frame:
            header.frame_id = self.target_frame
        return cloud, header


class Box(Filter):

    def __init__(self, keep=True, lower=None, upper=None, fields=('x', 'y', 'z'), frame=None, timeout=0.0):
        self.keep = keep
        self.lower = col(lower) if lower is not None else None
        self.upper = col(upper) if upper is not None else None
        self.fields = fields
        self.transform = Transform(frame, transform=[self.fields], timeout=timeout) if frame else PassThrough()

    def filter(self, cloud, header):
        cloud = cloud.ravel()
        filter_cloud = self.transform(cloud[self.fields].copy(), header)
        x = np.stack(filter_cloud[f] for f in self.fields)
        keep = np.ones(x.shape, dtype=np.bool)
        if self.lower is not None:
            keep &= (x >= self.lower).all(axis=0, keepdims=True)
        if self.upper is not None:
            keep &= (x <= self.upper).all(axis=0, keepdims=True)
        keep = keep if self.keep else ~keep
        cloud = cloud[keep]
        return cloud, header


class KeepFields(Filter):

    def __init__(self, fields=('x', 'y', 'z')):
        self.fields = fields

    def filter(self, cloud, header):
        cloud = cloud[self.fields]
        return cloud, header


class DiscardOld(Filter):

    def __init__(self, max_age=None):
        self.max_age = max_age

    def filter(self, cloud, header):
        age = (rospy.Time.now() - header.stamp).to_sec()
        if age > self.max_age:
            rospy.loginfo('Discarding old message (%.3f s > %.3f s).', age, self.max_age)
            return None, None
        return cloud, header


class FilterChain(Filter):

    def __init__(self, filters=()):
        self.filters = list(filters)

    def filter(self, cloud, header):
        t0 = timer()
        for i, f in enumerate(self.filters):
            n0 = cloud.size
            cloud, header = f(cloud, header)
            if cloud is None:
                rospy.loginfo('Cloud dropped in filter %s (%i).' % (f, i))
                break
            n1 = cloud.size
            if n1 < n0:
                rospy.loginfo('%i (%.2f%%) points removed by filter %s (%i).' % (n0 - n1, (n0 - n1) / n0, f, i))
        rospy.loginfo('Cloud processed (%.3f s).' % (timer() - t0))
        return cloud, header
