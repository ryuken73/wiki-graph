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
    font-weight: 300;
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
export const LinkCountContainer = styled.div`
  display: flex;
`
export const LinkCount = styled(Action)`
  font-size: 11px;
  font-weight: 200;
  color: ${props => props.disabled ? 'dimgrey':'yellow'};
  text-align: right;
  cursor: ${props => !props.disabled && 'pointer'};
`