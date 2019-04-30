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

type Message struct {
	ImageBase64 string
	Timestamp   uint64
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
	body, _ := ioutil.ReadAll(r.Body)
	m := Message{}

	err := json.Unmarshal(body, &m)
	if err != nil {
		log.Println(err)
		return
	}
	base64Dec := base64.NewDecoder(base64.StdEncoding, strings.NewReader(m.ImageBase64))
	imgBody, _ := ioutil.ReadAll(base64Dec)
	_, err = jpeg.Decode(bytes.NewReader(imgBody))
	if err != nil {
		log.Println(err)
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
