import face_recognition
import cv2 as cv

cap = cv.VideoCapture(0)


counter=0#######
face_locations = []######


while 1:
    ret, frame = cap.read()
    if ret:

        # Find all the faces in the image
        face_locations = face_recognition.face_locations(frame)
      
        #counter=counter+1
        #if counter==3:
            #face_locations = face_recognition.face_locations(frame)
            #counter=0

        number_of_faces = len(face_locations)
        print("I found {} face(s) in this photograph.".format(number_of_faces))

        for face_location in face_locations:
            # Print the location of each face in this image. Each face is a list of co-ordinates in (top, right, bottom, left) order.
            top, right, bottom, left = face_location
            print(
                "A face is located at pixel location Top: {}, Left: {}, Bottom: {}, Right: {}".format(top, left, bottom,
                                                                                                      right))
            # Let's draw a box around the face
            cv.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

        cv.imshow('img', frame)
        k = cv.waitKey(60) & 0xff
        if k == 27:
            break
    else:
        break

cv.destroyAllWindows()
cap.release()


