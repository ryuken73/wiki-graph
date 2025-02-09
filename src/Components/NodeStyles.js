import styled from "styled-components";

export const RowContainer = styled.div`
  padding: 3px;
  background-color: black;
  margin-bottom: 3px;
`
export const Node = styled.div`
  display: flex;
  font-size: 12px;
  font-weight: 100;
  color: yellow;
`
export const Action = styled.div`
  margin-left: 3px;
`
export const Title = styled(Action)`
  /* color: ${(props) => props.action === 'del' && 'lightgrey'}; */
  cursor: pointer;
  width: 100%;
  white-space: nowrap;
  text-overflow: ellipsis;
  overflow: hidden;
  &:hover {
    color: white;
  };
`
export const DelButton = styled(Action)`
  color: red;
  margin-left: auto;
  font-weight: 200;
  cursor: pointer;
  &:hover {
    color: white;
  };
`
export const TimeStamp = styled(Action)`
  font-size: 11px;
  color: #d73232;
  text-align: right;
  cursor: pointer;
`