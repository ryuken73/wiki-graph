import { API_URL } from "./constants.js";

export const getBacklinksByContentId = async (contentId) => {
  const personData = await fetch(`${API_URL}/backlinks/byContentId/${contentId}`);
  const jsonResult = await personData.json();
  const {rows, rowCount} = jsonResult;
  console.log(rows)
  return rows
}
export const getForwardlinksByBacklinkId = async (backlinkId) => {
  const personData = await fetch(`${API_URL}/forwardlinks/byBacklinkId/${backlinkId}`);
  const jsonResult = await personData.json();
  const {rows, rowCount} = jsonResult;
  console.log(rows)
  return rows
}