// klippy/vision/vision.c
#include "Python.h"
#include "structmember.h"
#include <openpnp-capture.h>

typedef struct {
    PyObject_HEAD
    CapContext ctx;        // one context per Vision object
    CapStream stream;
    uint32_t width, height;
    uint8_t *buffer;
} VisionCamera;

static void VisionCamera_dealloc(VisionCamera *self) {
    if (self->stream) Cap_closeStream(self->ctx, self->stream);
    if (self->ctx) Cap_releaseContext(self->ctx);
    if (self->buffer) free(self->buffer);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *VisionCamera_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    VisionCamera *self = (VisionCamera *)type->tp_alloc(type, 0);
    if (self) {
        self->ctx = Cap_createContext();
        self->stream = 0;
        self->buffer = NULL;
    }
    return (PyObject *)self;
}

static int VisionCamera_init(VisionCamera *self, PyObject *args, PyObject *kwds) {
    const char *uid = NULL;
    int format_id = 11;
    static char *kwlist[] = {"camera_uid", "format_id", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|zi", kwlist, &uid, &format_id))
        return -1;

    uint32_t dev_idx = 0;
    if (uid) {
        uint32_t count = Cap_getDeviceCount(self->ctx);
        for (uint32_t i = 0; i < count; i++) {
            const char *dev_uid = Cap_getDeviceUniqueID(self->ctx, i);
            if (dev_uid && strcmp(dev_uid, uid) == 0) { dev_idx = i; break; }
        }
    }

    CapFormatInfo info;
    if (Cap_getFormatInfo(self->ctx, dev_idx, format_id, &info) != CAPRESULT_OK)
        return -1;

    self->width = info.width;
    self->height = info.height;
    self->buffer = malloc(info.width * info.height * 3);
    if (!self->buffer) return -1;

    self->stream = Cap_openStream(self->ctx, dev_idx, format_id);
    if (!self->stream) return -1;

    return 0;
}

static PyObject *VisionCamera_snap(VisionCamera *self, PyObject *Py_UNUSED(ignored)) {
    if (!self->stream) Py_RETURN_NONE;
    CapResult r = Cap_captureFrame(self->ctx, self->stream,
                                   self->buffer, self->width * self->height * 3);
    if (r != CAPRESULT_OK) {
        PyErr_Format(PyExc_RuntimeError, "Capture failed: %d", r);
        return NULL;
    }
    return Py_BuildValue("iiy#", self->width, self->height,
                         self->buffer, self->width * self->height * 3);
}

static PyMethodDef VisionCamera_methods[] = {
    {"snap", (PyCFunction)VisionCamera_snap, METH_NOARGS, "Capture RGB frame"},
    {NULL}
};

static PyTypeObject VisionCameraType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "vision.VisionCamera",
    .tp_basicsize = sizeof(VisionCamera),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_dealloc = (destructor)VisionCamera_dealloc,
    .tp_new = VisionCamera_new,
    .tp_init = (initproc)VisionCamera_init,
    .tp_methods = VisionCamera_methods,
};

static struct PyModuleDef visionmodule = {
    PyModuleDef_HEAD_INIT,
    "vision",
    "Native openpnp-capture camera module",
    -1,
};

PyMODINIT_FUNC PyInit_vision(void) {
    PyObject *m = PyModule_Create(&visionmodule);
    if (!m) return NULL;
    if (PyType_Ready(&VisionCameraType) < 0) return NULL;
    Py_INCREF(&VisionCameraType);
    PyModule_AddObject(m, "VisionCamera", (PyObject *)&VisionCameraType);
    return m;
}
