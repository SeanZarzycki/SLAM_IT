package main

import (
	"bytes"
	"encoding/base64"
	"encoding/json"
	"fmt"
	"image/jpeg"
	"io/ioutil"
	"log"
	"net/http"
	"strconv"
	"strings"
)

const ImageDir = "./images/"

// Message is what is received from the phone and sent to the server
type Message struct {
	ImageBase64 string
	Timestamp   uint64
}

// ParseImage checks to see if Message m is ok. returns err from jpeg
func (m Message) ParseImage() (imgBody []byte, err error) {

	return
}

// adds the ".jpg" extension to a uint
func uint2jpegfile(x uint64) (filename string) {
	filename = fmt.Sprintf("%v.jpg", x)
	return
}

func highestFileNameValue() (maxVal uint64) {
	maxVal = 0
	files, err := ioutil.ReadDir(ImageDir)
	if err != nil {
		log.Println(err)
		return
	}
	for _, f := range files {
		fnameSplit := strings.Split(f.Name(), ".")
		baseName := fnameSplit[0]
		val, err := strconv.ParseUint(baseName, 10, 64)
		if err == nil && val > maxVal {
			maxVal = val
		}
	}
	return
}

func handler(w http.ResponseWriter, r *http.Request) {
	// jsonEnc := json.NewEncoder(os.Stdout)
	log.Println("Handler hit")
	body, _ := ioutil.ReadAll(r.Body)
	m := Message{}
	err := json.Unmarshal(body, &m)
	if err != nil {
		log.Println("Bad json >:(")
		log.Println(body)
		log.Println(err)
		return
	}
	log.Println("Good json :D")
	base64Dec := base64.NewDecoder(base64.StdEncoding, strings.NewReader(m.ImageBase64))
	imgBody, _ := ioutil.ReadAll(base64Dec)
	_, err = jpeg.Decode(bytes.NewReader(imgBody))
	if err != nil {
		log.Println("Bad jpeg >:(")
		log.Println(err)
		ioutil.WriteFile("failed.jpg", imgBody, 0664)
		ioutil.WriteFile("failed.b64", []byte(m.ImageBase64), 0664)
		return
	}

	filename := uint2jpegfile(highestFileNameValue() + 1)
	log.Println("Writing to " + filename)
	ioutil.WriteFile(ImageDir+filename, imgBody, 0664)

	// jsonEnc.Encode(&m)
}

func main() {
	http.HandleFunc("/upload", handler)
	log.Fatal(http.ListenAndServe(":8080", nil))
}
