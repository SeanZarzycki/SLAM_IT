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
	"os"
	"strconv"
	"strings"
)

const ImageDir = "./images/"

type Message struct {
	ImageBase64 string
	Timestamp   uint64
}

// exists returns whether the given file or directory exists
func exists(path string) (bool, error) {
	_, err := os.Stat(path)
	if err == nil {
		return true, nil
	}
	if os.IsNotExist(err) {
		return false, nil
	}
	return true, err
}

// adds the ".jpg" extension to a uint
func uint2jpegfile(x uint64) (filename string) {
	filename = fmt.Sprintf("%06v.jpg", x)
	return
}

func highestFileNameValue(dir string) (maxVal uint64) {
	maxVal = 0
	files, err := ioutil.ReadDir(dir)
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
	addr := strings.Split(r.RemoteAddr, ":")
	ip := addr[0]
	dir := ImageDir + ip
	dirExists, _ := exists(dir)
	if !dirExists {
		os.MkdirAll(dir, 0777)
	}

	filename := uint2jpegfile(highestFileNameValue(dir) + 1)

	path := dir + "/" + filename
	log.Println("Writing to " + path)
	ioutil.WriteFile(path, imgBody, 0664)

	// jsonEnc.Encode(&m)
}

func main() {
	http.HandleFunc("/upload", handler)
	log.Fatal(http.ListenAndServe(":8080", nil))
}
