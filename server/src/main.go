package main

import (
	"bufio"
	"bytes"
	"encoding/base64"
	"encoding/json"
	"fmt"
	"image/jpeg"
	"io/ioutil"
	"log"
	"net"
	"net/http"
	"os"
	"strconv"
	"strings"
)

const (
	ConnHost = "localhost"
	ConnPort = "8081"
	ConnType = "tcp"
	ImageDir = "./images/"
)

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

func handleRequest(c net.Conn) {
	for {
		netData, err := bufio.NewReader(c).ReadString('\n')
		if err != nil {
			fmt.Println(err)
			return
		}

		m := Message{}

		err = json.Unmarshal([]byte(netData), &m)
		if err != nil {
			log.Println(err)
		} else {
			base64Dec := base64.NewDecoder(base64.StdEncoding, strings.NewReader(m.ImageBase64))
			imgBody, _ := ioutil.ReadAll(base64Dec)
			_, err = jpeg.Decode(bytes.NewReader(imgBody))
			if err != nil {
				log.Println(err)
			} else {
				addr := strings.Split(c.RemoteAddr().String(), ":")
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
			}
		}

	}
}

func main() {
	l, err := net.Listen(ConnType, ":8081")
	if err != nil {
		fmt.Println("Error listening:", err.Error())
		os.Exit(1)
	}
	defer l.Close()
	fmt.Println("Listening on " + ConnHost + ":" + ConnPort)
	for {
		conn, err := l.Accept()
		if err != nil {
			fmt.Println("Error accepting: ", err.Error())
			os.Exit(1)
		}
		go handleRequest(conn)
	}
}
