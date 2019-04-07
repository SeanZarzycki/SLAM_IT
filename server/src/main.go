package main

import (
	"encoding/json"
	"fmt"
	"io/ioutil"
	"log"
	"net/http"
	"os"
)

type Message struct {
	ImageBase64 string
}

func handler(w http.ResponseWriter, r *http.Request) {
	enc := json.NewEncoder(os.Stdout)
	body, _ := ioutil.ReadAll(r.Body)
	m := Message{}
	err := json.Unmarshal(body, &m)
	if err != nil {
		fmt.Println(err)
		return
	}
	enc.Encode(&m)
}

func main() {
	http.HandleFunc("/upload", handler)
	log.Fatal(http.ListenAndServe(":8080", nil))
}
