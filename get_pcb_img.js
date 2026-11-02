
await eda.pcb_Document.zoomToBoardOutline();
await new Promise(r => setTimeout(r, 1000));
const blob = await eda.dmt_EditorControl.getCurrentRenderedAreaImage();
const buffer = await blob.arrayBuffer();
let binary = '';
const bytes = new Uint8Array(buffer);
const len = bytes.byteLength;
for (let i = 0; i < len; i++) {
    binary += String.fromCharCode(bytes[i]);
}
return btoa(binary);
