from pypylon import pylon
import cv2

class Basler():
    def __init__(self):
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()

        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def close(self):
        self.camera.StopGrabbing()
        self.camera.Close()

if __name__=="__main__":
    basler = Basler()
    while basler.camera.IsGrabbing():
        grab_result = basler.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        if grab_result.GrabSucceeded():
            img = basler.converter.Convert(grab_result)
            frame = img.GetArray()

            cv2.namedWindow("Basler Live", cv2.WINDOW_NORMAL)
            cv2.imshow("Basler Live", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        grab_result.Release()

    basler.close()
    cv2.destroyAllWindows()
